import Foundation

struct DatagramEndpoint: Equatable, Sendable {
  let host: String
  let port: UInt16
}

enum DatagramTransportState: Equatable, Sendable {
  case disconnected
  case preparing
  case ready
  case waiting(String)
  case failed(String)
}

protocol DatagramTransport: AnyObject, Sendable {
  var replacedPoseCount: UInt64 { get }

  func start(
    endpoint: DatagramEndpoint,
    receiveHandler: @escaping @Sendable (Data) -> Void,
    stateHandler: @escaping @Sendable (DatagramTransportState) -> Void
  )
  func sendPose(_ payload: Data)
  func sendControl(_ payload: Data)
  func stop(finalDatagram: Data?)
}

final class LatestOnlyDatagramSender: @unchecked Sendable {
  typealias Completion = @Sendable (String?) -> Void
  typealias SendOperation = @Sendable (Data, @escaping Completion) -> Void

  private let sendOperation: SendOperation
  private let failureHandler: @Sendable (String) -> Void
  private let lock = NSLock()
  private var active = true
  private var inFlight = false
  private var pending: Data?
  private var replacements: UInt64 = 0

  init(
    sendOperation: @escaping SendOperation,
    failureHandler: @escaping @Sendable (String) -> Void = { _ in }
  ) {
    self.sendOperation = sendOperation
    self.failureHandler = failureHandler
  }

  func submit(_ payload: Data) {
    let shouldSend = lock.withLock {
      guard active else { return false }
      if inFlight {
        if pending != nil {
          replacements &+= 1
        }
        pending = payload
        return false
      }
      inFlight = true
      return true
    }
    if shouldSend {
      performSend(payload)
    }
  }

  func stop() {
    lock.withLock {
      active = false
      pending = nil
    }
  }

  var replacementCount: UInt64 {
    lock.withLock { replacements }
  }

  var hasPendingWork: Bool {
    lock.withLock { inFlight || pending != nil }
  }

  private func performSend(_ payload: Data) {
    sendOperation(payload) { [weak self] error in
      self?.complete(error: error)
    }
  }

  private func complete(error: String?) {
    var next: Data?
    let shouldReportFailure = lock.withLock {
      guard active else {
        inFlight = false
        pending = nil
        return false
      }
      if error != nil {
        inFlight = false
        pending = nil
        return true
      }
      if let pending {
        next = pending
        self.pending = nil
      } else {
        inFlight = false
      }
      return false
    }

    if shouldReportFailure, let error {
      failureHandler(error)
    } else if let next {
      performSend(next)
    }
  }
}
