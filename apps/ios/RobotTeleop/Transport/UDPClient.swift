import Foundation
@preconcurrency import Network

final class UDPClient: DatagramTransport, @unchecked Sendable {
  private let queue = DispatchQueue(
    label: "com.mo.robotteleop.transport.udp",
    qos: .userInteractive
  )
  private let senderLock = NSLock()

  private var generation: UInt64 = 0
  private var connection: NWConnection?
  private var poseSender: LatestOnlyDatagramSender?
  private var receiveHandler: (@Sendable (Data) -> Void)?
  private var stateHandler: (@Sendable (DatagramTransportState) -> Void)?
  private var state = DatagramTransportState.disconnected

  var replacedPoseCount: UInt64 {
    senderLock.withLock { poseSender?.replacementCount ?? 0 }
  }

  func start(
    endpoint: DatagramEndpoint,
    receiveHandler: @escaping @Sendable (Data) -> Void,
    stateHandler: @escaping @Sendable (DatagramTransportState) -> Void
  ) {
    queue.async { [weak self] in
      self?.startOnQueue(
        endpoint: endpoint,
        receiveHandler: receiveHandler,
        stateHandler: stateHandler
      )
    }
  }

  func sendPose(_ payload: Data) {
    queue.async { [weak self] in
      guard let self, state == .ready else { return }
      senderLock.withLock { poseSender }?.submit(payload)
    }
  }

  func sendControl(_ payload: Data) {
    queue.async { [weak self] in
      guard let self, state == .ready, let connection else { return }
      let activeGeneration = generation
      connection.send(
        content: payload,
        completion: .contentProcessed { [weak self] error in
          guard let error else { return }
          self?.queue.async { [weak self] in
            self?.failOnQueue(error.localizedDescription, generation: activeGeneration)
          }
        }
      )
    }
  }

  func stop(finalDatagram: Data?) {
    queue.async { [weak self] in
      self?.stopOnQueue(finalDatagram: finalDatagram)
    }
  }

  private func startOnQueue(
    endpoint: DatagramEndpoint,
    receiveHandler: @escaping @Sendable (Data) -> Void,
    stateHandler: @escaping @Sendable (DatagramTransportState) -> Void
  ) {
    finishStopOnQueue(emitDisconnected: false)
    let host = endpoint.host.trimmingCharacters(in: .whitespacesAndNewlines)
    guard !host.isEmpty, endpoint.port > 0,
      let port = NWEndpoint.Port(rawValue: endpoint.port)
    else {
      stateHandler(.failed("Invalid UDP endpoint"))
      return
    }

    generation &+= 1
    let activeGeneration = generation
    self.receiveHandler = receiveHandler
    self.stateHandler = stateHandler
    transition(to: .preparing)

    let connection = NWConnection(
      host: NWEndpoint.Host(host),
      port: port,
      using: .udp
    )
    self.connection = connection
    let sender = LatestOnlyDatagramSender(
      sendOperation: { [weak connection] payload, completion in
        guard let connection else {
          completion("UDP connection is unavailable")
          return
        }
        connection.send(
          content: payload,
          completion: .contentProcessed { error in
            completion(error?.localizedDescription)
          }
        )
      },
      failureHandler: { [weak self] error in
        self?.queue.async { [weak self] in
          self?.failOnQueue(error, generation: activeGeneration)
        }
      }
    )
    senderLock.withLock {
      poseSender = sender
    }

    connection.stateUpdateHandler = { [weak self] networkState in
      self?.handleNetworkState(networkState, generation: activeGeneration)
    }
    connection.start(queue: queue)
  }

  private func handleNetworkState(_ networkState: NWConnection.State, generation: UInt64) {
    guard generation == self.generation else { return }
    switch networkState {
    case .setup:
      break
    case .preparing:
      transition(to: .preparing)
    case .ready:
      transition(to: .ready)
      receiveNext(generation: generation)
    case .waiting(let error):
      transition(to: .waiting(error.localizedDescription))
    case .failed(let error):
      failOnQueue(error.localizedDescription, generation: generation)
    case .cancelled:
      finishStopOnQueue(emitDisconnected: true)
    @unknown default:
      failOnQueue("Unknown UDP connection state", generation: generation)
    }
  }

  private func receiveNext(generation: UInt64) {
    guard generation == self.generation, let connection else { return }
    connection.receiveMessage { [weak self] content, _, _, error in
      guard let self, generation == self.generation else { return }
      if let content, !content.isEmpty {
        receiveHandler?(content)
      }
      if let error {
        failOnQueue(error.localizedDescription, generation: generation)
      } else if state == .ready {
        receiveNext(generation: generation)
      }
    }
  }

  private func stopOnQueue(finalDatagram: Data?) {
    let activeGeneration = generation
    let sender = senderLock.withLock { () -> LatestOnlyDatagramSender? in
      let sender = poseSender
      poseSender = nil
      return sender
    }
    sender?.stop()

    guard let connection else {
      finishStopOnQueue(emitDisconnected: true)
      return
    }
    guard let finalDatagram, state == .ready else {
      finishStopOnQueue(emitDisconnected: true)
      return
    }

    connection.send(
      content: finalDatagram,
      completion: .contentProcessed { [weak self, weak connection] _ in
        guard let self, let connection else { return }
        self.queue.async { [weak self, weak connection] in
          guard let self, let connection else { return }
          self.finishStopOnQueue(
            connection: connection,
            generation: activeGeneration,
            emitDisconnected: true
          )
        }
      }
    )
    queue.asyncAfter(deadline: .now() + .milliseconds(100)) { [weak self, weak connection] in
      guard let self, let connection else { return }
      self.finishStopOnQueue(
        connection: connection,
        generation: activeGeneration,
        emitDisconnected: true
      )
    }
  }

  private func failOnQueue(_ message: String, generation: UInt64) {
    guard generation == self.generation else { return }
    transition(to: .failed(message))
    finishStopOnQueue(emitDisconnected: false)
  }

  private func finishStopOnQueue(
    connection expectedConnection: NWConnection? = nil,
    generation expectedGeneration: UInt64? = nil,
    emitDisconnected: Bool
  ) {
    if let expectedGeneration, expectedGeneration != generation { return }
    if let expectedConnection, expectedConnection !== connection { return }

    generation &+= 1
    let oldConnection = connection
    connection = nil
    oldConnection?.stateUpdateHandler = nil
    oldConnection?.cancel()
    let sender = senderLock.withLock { () -> LatestOnlyDatagramSender? in
      let sender = poseSender
      poseSender = nil
      return sender
    }
    sender?.stop()

    if emitDisconnected {
      transition(to: .disconnected)
    }
    receiveHandler = nil
    stateHandler = nil
  }

  private func transition(to newState: DatagramTransportState) {
    guard state != newState else { return }
    state = newState
    stateHandler?(newState)
  }
}
