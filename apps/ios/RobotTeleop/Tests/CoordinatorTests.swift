import Foundation
import XCTest

@testable import RobotTeleop

private final class ManualPoseProvider: PoseProvider, @unchecked Sendable {
  let kind = PoseProviderKind.simulated

  private let lock = NSLock()
  private var handler: (@Sendable (PoseSample) -> Void)?
  private(set) var stopCount = 0

  func start(handler: @escaping @Sendable (PoseSample) -> Void) {
    lock.withLock {
      self.handler = handler
    }
  }

  func stop() {
    lock.withLock {
      handler = nil
      stopCount += 1
    }
  }

  func emit(_ sample: PoseSample) {
    let callback = lock.withLock { handler }
    callback?(sample)
  }
}

private final class MockDatagramTransport: DatagramTransport, @unchecked Sendable {
  private let lock = NSLock()
  private var receiveHandler: (@Sendable (Data) -> Void)?
  private var stateHandler: (@Sendable (DatagramTransportState) -> Void)?
  private var poses: [Data] = []
  private var controls: [Data] = []
  private var finalDatagrams: [Data] = []
  private var kinds: [SentDatagramKind] = []

  private(set) var endpoint: DatagramEndpoint?

  var replacedPoseCount: UInt64 { 0 }

  func start(
    endpoint: DatagramEndpoint,
    receiveHandler: @escaping @Sendable (Data) -> Void,
    stateHandler: @escaping @Sendable (DatagramTransportState) -> Void
  ) {
    lock.withLock {
      self.endpoint = endpoint
      self.receiveHandler = receiveHandler
      self.stateHandler = stateHandler
    }
  }

  func sendPose(_ payload: Data) {
    lock.withLock {
      poses.append(payload)
      kinds.append(.pose)
    }
  }

  func sendControl(_ payload: Data) {
    lock.withLock {
      controls.append(payload)
      kinds.append(.control)
    }
  }

  func stop(finalDatagram: Data?) {
    let callback = lock.withLock {
      if let finalDatagram {
        finalDatagrams.append(finalDatagram)
      }
      return stateHandler
    }
    callback?(.disconnected)
  }

  func transition(to state: DatagramTransportState) {
    let callback = lock.withLock { stateHandler }
    callback?(state)
  }

  func emit(_ data: Data) {
    let callback = lock.withLock { receiveHandler }
    callback?(data)
  }

  var sentPoses: [Data] {
    lock.withLock { poses }
  }

  var sentControls: [Data] {
    lock.withLock { controls }
  }

  var finals: [Data] {
    lock.withLock { finalDatagrams }
  }

  var sentKinds: [SentDatagramKind] {
    lock.withLock { kinds }
  }
}

private enum SentDatagramKind: Equatable {
  case pose
  case control
}

private final class SessionSequence: @unchecked Sendable {
  private let lock = NSLock()
  private var values: [UInt32]

  init(_ values: [UInt32]) {
    self.values = values
  }

  func next() -> UInt32 {
    lock.withLock { values.removeFirst() }
  }
}

private final class ManualClock: @unchecked Sendable {
  private let lock = NSLock()
  private var value: UInt64

  init(_ value: UInt64) {
    self.value = value
  }

  func now() -> UInt64 {
    lock.withLock { value }
  }

  func set(_ value: UInt64) {
    lock.withLock {
      self.value = value
    }
  }
}

private final class ManualDatagramSink: @unchecked Sendable {
  typealias Completion = @Sendable (String?) -> Void

  private let lock = NSLock()
  private var payloads: [Data] = []
  private var completions: [Completion] = []

  func send(_ payload: Data, completion: @escaping Completion) {
    lock.withLock {
      payloads.append(payload)
      completions.append(completion)
    }
  }

  func completeNext(error: String? = nil) {
    let completion = lock.withLock { completions.removeFirst() }
    completion(error)
  }

  var sent: [Data] {
    lock.withLock { payloads }
  }
}

private final class StatusCollector: @unchecked Sendable {
  private let lock = NSLock()
  private var values: [TeleopCoordinatorStatus] = []

  func append(_ value: TeleopCoordinatorStatus) {
    lock.withLock {
      values.append(value)
    }
  }

  var latest: TeleopCoordinatorStatus? {
    lock.withLock { values.last }
  }

  var count: Int {
    lock.withLock { values.count }
  }

  func clear() {
    lock.withLock {
      values.removeAll(keepingCapacity: true)
    }
  }
}

final class CoordinatorTests: XCTestCase {
  func testPhoneControlConfirmsOwnershipAndRejectsWrongRequestReplies() throws {
    let clock = ManualClock(1_000_000_000)
    let fixture = makeCoordinator(sessionIDs: [71], clock: clock.now)
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    fixture.provider.emit(sample(timestampNs: 1_000_000_000))
    XCTAssertTrue(waitUntil { !fixture.transport.sentControls.isEmpty })
    let request = try XCTUnwrap(JSONSerialization.jsonObject(with: fixture.transport.sentControls[0]) as? [String: Any])
    XCTAssertEqual(request["action"] as? String, "monitor")
    XCTAssertFalse(fixture.status.latest?.controlAvailable ?? true)
    var reply: [String: Any] = ["protocol": "tia.phone.control.v1", "id": 999, "session_id": 71,
      "ok": true, "message": "", "kernel_runtime_id": "test", "enabled": false,
      "active": false, "estop": false, "target_limited": false, "robot_gripper": 0.4]
    fixture.transport.emit(try JSONSerialization.data(withJSONObject: reply))
    XCTAssertTrue(waitUntil { (fixture.status.latest?.invalidResponseCount ?? 0) == 1 })
    reply["id"] = request["id"]
    fixture.transport.emit(try JSONSerialization.data(withJSONObject: reply))
    XCTAssertTrue(waitUntil { fixture.status.latest?.controlAvailable == true })
    fixture.coordinator.requestControl("enable")
    XCTAssertTrue(waitUntil { fixture.transport.sentControls.count == 2 })
    let enable = try XCTUnwrap(JSONSerialization.jsonObject(with: fixture.transport.sentControls[1]) as? [String: Any])
    XCTAssertEqual(enable["runtime_id"] as? String, "test")
    reply["id"] = enable["id"]
    reply["enabled"] = true
    fixture.transport.emit(try JSONSerialization.data(withJSONObject: reply))
    XCTAssertTrue(waitUntil { fixture.status.latest?.phoneControl?.enabled == true })
    for action in ["home", "pregrasp"] {
      let count = fixture.transport.sentControls.count
      fixture.coordinator.requestControl(action)
      XCTAssertTrue(waitUntil { fixture.transport.sentControls.count == count + 1 })
      let preset = try XCTUnwrap(JSONSerialization.jsonObject(with: fixture.transport.sentControls[count]) as? [String: Any])
      XCTAssertEqual(preset["action"] as? String, action)
      XCTAssertEqual(preset["runtime_id"] as? String, "test")
      reply["id"] = preset["id"]
      reply["preset"] = ["name": action, "phase": "opening", "message": "先张开夹爪"]
      fixture.transport.emit(try JSONSerialization.data(withJSONObject: reply))
      XCTAssertTrue(waitUntil { fixture.status.latest?.phoneControl?.preset?.name == action })
      XCTAssertEqual(fixture.status.latest?.phoneControl?.preset?.busy, true)
    }
    let count = fixture.transport.sentControls.count
    fixture.coordinator.requestControl("disable")
    XCTAssertTrue(waitUntil { fixture.transport.sentControls.count == count + 1 })
    let stop = try XCTUnwrap(JSONSerialization.jsonObject(with: fixture.transport.sentControls[count]) as? [String: Any])
    XCTAssertEqual(stop["action"] as? String, "disable")
    fixture.coordinator.disconnect()
  }

  func testLatestOnlySenderOverwritesPendingPose() {
    let sink = ManualDatagramSink()
    let sender = LatestOnlyDatagramSender(sendOperation: sink.send)

    sender.submit(Data([1]))
    sender.submit(Data([2]))
    sender.submit(Data([3]))

    XCTAssertEqual(sink.sent, [Data([1])])
    XCTAssertEqual(sender.replacementCount, 1)

    sink.completeNext()
    XCTAssertEqual(sink.sent, [Data([1]), Data([3])])
    sink.completeNext()
    XCTAssertFalse(sender.hasPendingWork)
  }

  func testCoordinatorSendsOnePoseForEachNewSample() throws {
    let fixture = makeCoordinator(sessionIDs: [11])
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)

    fixture.provider.emit(sample(timestampNs: 100))
    fixture.provider.emit(sample(timestampNs: 200))

    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 2 })
    let decoded = try fixture.transport.sentPoses.map(PacketCodec.decode)
    let poses = decoded.compactMap { packet -> PosePacket? in
      guard case .pose(let pose) = packet else { return nil }
      return pose
    }
    XCTAssertEqual(poses.map(\.sequence), [0, 1])
    XCTAssertEqual(poses.map(\.sessionID), [11, 11])
  }

  func testTrackingLossReleasesClutchOnTheNextSample() throws {
    let fixture = makeCoordinator(sessionIDs: [21])
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    fixture.provider.emit(sample(timestampNs: 100, tracking: .normal))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 1 })
    fixture.coordinator.setClutchActive(true)
    fixture.provider.emit(sample(timestampNs: 200, tracking: .normal))
    fixture.provider.emit(sample(timestampNs: 300, tracking: .unavailable))

    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 3 })
    let baseline = try pose(from: fixture.transport.sentPoses[0])
    let active = try pose(from: fixture.transport.sentPoses[1])
    let trackingLost = try pose(from: fixture.transport.sentPoses[2])
    XCTAssertFalse(baseline.clutchActive)
    XCTAssertTrue(active.clutchActive)
    XCTAssertFalse(trackingLost.clutchActive)
    XCTAssertFalse(fixture.controls.snapshot.clutchActive)
  }

  func testExplicitClutchReleaseSendsImmediateMonotonicSafetyPose() throws {
    let fixture = makeCoordinator(sessionIDs: [22], clock: { 250 })
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    fixture.provider.emit(sample(timestampNs: 100))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 1 })
    fixture.coordinator.setClutchActive(true)
    fixture.provider.emit(sample(timestampNs: 200))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 2 })

    fixture.coordinator.setClutchActive(false)

    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 3 })
    let active = try pose(from: fixture.transport.sentPoses[1])
    let released = try pose(from: fixture.transport.sentPoses[2])
    XCTAssertTrue(active.clutchActive)
    XCTAssertFalse(released.clutchActive)
    XCTAssertGreaterThan(released.clientTimeNs, active.clientTimeNs)
  }

  func testEstopSendsImmediateLatchedSafetyPose() throws {
    let fixture = makeCoordinator(sessionIDs: [23], clock: { 150 })
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    fixture.provider.emit(sample(timestampNs: 100))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 1 })

    fixture.coordinator.latchEstop()

    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 2 })
    let estop = try pose(from: fixture.transport.sentPoses[1])
    XCTAssertTrue(estop.estopRequested)
    XCTAssertFalse(estop.clutchActive)
  }

  func testHomeIsOneShotAndNextClutchCanControl() throws {
    let fixture = makeCoordinator(sessionIDs: [24])
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    fixture.provider.emit(sample(timestampNs: 100))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 1 })

    fixture.coordinator.setClutchActive(true)
    fixture.provider.emit(sample(timestampNs: 200))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 2 })
    fixture.coordinator.requestHome()
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 3 })
    fixture.provider.emit(sample(timestampNs: 300))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 4 })
    fixture.coordinator.setClutchActive(true)
    fixture.provider.emit(sample(timestampNs: 400))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 5 })

    let active = try pose(from: fixture.transport.sentPoses[1])
    let home = try pose(from: fixture.transport.sentPoses[2])
    let releasedAfterHome = try pose(from: fixture.transport.sentPoses[3])
    let activeAfterHome = try pose(from: fixture.transport.sentPoses[4])
    XCTAssertTrue(active.clutchActive)
    XCTAssertTrue(home.homeRequested)
    XCTAssertFalse(home.clutchActive)
    XCTAssertFalse(releasedAfterHome.homeRequested)
    XCTAssertFalse(releasedAfterHome.clutchActive)
    XCTAssertFalse(activeAfterHome.homeRequested)
    XCTAssertTrue(activeAfterHome.clutchActive)
  }

  func testEstopResetUnlocksOnlyAfterAMatchingAcknowledgement() throws {
    let fixture = makeCoordinator(sessionIDs: [24], clock: { 500 })
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    fixture.provider.emit(sample(timestampNs: 100))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 1 })

    fixture.coordinator.latchEstop()
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 2 })
    fixture.coordinator.requestEstopReset()
    XCTAssertTrue(waitUntil { fixture.transport.sentControls.count == 1 })
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 3 })

    let request = try estopResetRequest(from: fixture.transport.sentControls[0])
    let released = try pose(from: fixture.transport.sentPoses[2])
    XCTAssertFalse(released.estopRequested)
    XCTAssertFalse(released.clutchActive)
    XCTAssertTrue(fixture.controls.snapshot.estopResetPending)
    XCTAssertEqual(fixture.status.latest?.estopResetState, .pending)

    fixture.transport.emit(
      try PacketCodec.encode(
        .estopResetAck(
          EStopResetAckPacket(
            sourceType: .mock,
            sequence: request.sequence,
            sessionID: request.sessionID,
            clientTimeNs: request.clientTimeNs
          )
        )
      )
    )
    XCTAssertTrue(waitUntil { (fixture.status.latest?.invalidResponseCount ?? 0) == 1 })
    XCTAssertTrue(fixture.controls.snapshot.estopResetPending)

    fixture.transport.emit(
      try PacketCodec.encode(
        .estopResetAck(
          EStopResetAckPacket(
            sequence: request.sequence,
            sessionID: request.sessionID,
            clientTimeNs: request.clientTimeNs
          )
        )
      )
    )
    XCTAssertTrue(waitUntil { fixture.status.latest?.estopResetState == .acknowledged })
    XCTAssertFalse(fixture.controls.snapshot.estopRequested)
    XCTAssertFalse(fixture.controls.snapshot.estopResetPending)
  }

  func testDisconnectRestoresPendingResetToALatchedEstop() throws {
    let fixture = makeCoordinator(sessionIDs: [25], clock: { 500 })
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    fixture.provider.emit(sample(timestampNs: 100))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 1 })
    fixture.coordinator.latchEstop()
    fixture.coordinator.requestEstopReset()
    XCTAssertTrue(waitUntil { fixture.controls.snapshot.estopResetPending })

    fixture.coordinator.disconnect()

    XCTAssertTrue(waitUntil { fixture.transport.finals.count == 1 })
    XCTAssertTrue(fixture.controls.snapshot.estopRequested)
    XCTAssertFalse(fixture.controls.snapshot.estopResetPending)
    XCTAssertTrue(try pose(from: fixture.transport.finals[0]).estopRequested)
  }

  func testReconnectCreatesANewSessionAndFinalPacketIsReleased() throws {
    let fixture = makeCoordinator(sessionIDs: [31, 32])
    let endpoint = DatagramEndpoint(host: "127.0.0.1", port: 5005)
    fixture.coordinator.connect(endpoint: endpoint)
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    fixture.coordinator.setClutchActive(true)
    fixture.provider.emit(sample(timestampNs: 100))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 1 })
    XCTAssertFalse(try pose(from: fixture.transport.sentPoses[0]).clutchActive)
    fixture.provider.emit(sample(timestampNs: 200))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 2 })
    XCTAssertTrue(try pose(from: fixture.transport.sentPoses[1]).clutchActive)

    fixture.coordinator.disconnect()
    XCTAssertTrue(waitUntil { fixture.transport.finals.count == 1 })
    let finalPose = try pose(from: fixture.transport.finals[0])
    XCTAssertEqual(finalPose.sessionID, 31)
    XCTAssertFalse(finalPose.clutchActive)

    fixture.coordinator.connect(endpoint: endpoint)
    XCTAssertTrue(waitUntil { fixture.status.latest?.sessionID == 32 })
    fixture.transport.transition(to: .ready)
    fixture.provider.emit(sample(timestampNs: 300))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 3 })
    XCTAssertEqual(try pose(from: fixture.transport.sentPoses[2]).sessionID, 32)
    XCTAssertFalse(try pose(from: fixture.transport.sentPoses[2]).clutchActive)
  }

  func testPongAndSettingsAckUpdateStatus() throws {
    let clock = ManualClock(1_000)
    let fixture = makeCoordinator(sessionIDs: [41], clock: clock.now)
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    fixture.provider.emit(sample(timestampNs: 100))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 1 })
    XCTAssertTrue(fixture.transport.sentControls.isEmpty)
    fixture.provider.emit(sample(timestampNs: 200))
    XCTAssertTrue(waitUntil { fixture.transport.sentControls.count == 1 })
    XCTAssertEqual(fixture.transport.sentKinds, [.pose, .pose, .control])

    let settings = try settings(from: fixture.transport.sentControls[0])
    fixture.transport.emit(
      try PacketCodec.encode(
        .settingsAck(
          SettingsAckPacket(
            sequence: settings.sequence,
            sessionID: settings.sessionID,
            clientTimeNs: settings.clientTimeNs,
            positionScale: settings.positionScale,
            rotationScale: settings.rotationScale
          )
        )
      )
    )
    fixture.coordinator.requestPing()
    fixture.provider.emit(sample(timestampNs: 300))
    XCTAssertTrue(waitUntil { fixture.transport.sentControls.count == 2 })
    XCTAssertEqual(
      Array(fixture.transport.sentKinds.suffix(2)),
      [.pose, .control]
    )
    let ping = try ping(from: fixture.transport.sentControls[1])
    clock.set(1_040)
    fixture.transport.emit(
      try PacketCodec.encode(
        .pong(
          PongPacket(
            sequence: ping.sequence,
            sessionID: ping.sessionID,
            clientTimeNs: ping.clientTimeNs,
            serverReceiveTimeNs: 1_010,
            serverSendTimeNs: 1_020
          )
        )
      )
    )

    XCTAssertTrue(
      waitUntil {
        fixture.status.latest?.settingsState == .acknowledged
          && fixture.status.latest?.rttMs == 0.000_03
      }
    )
  }

  func testStatusIsCoalescedWithoutDroppingPosePackets() {
    let clock = ManualClock(0)
    let fixture = makeCoordinator(
      sessionIDs: [51],
      clock: clock.now,
      statusPublishIntervalNs: 100
    )
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    clock.set(1)
    fixture.provider.emit(sample(timestampNs: 100))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 1 })
    clock.set(2)
    fixture.provider.emit(sample(timestampNs: 200))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 2 })
    XCTAssertTrue(waitUntil { fixture.status.latest?.settingsState == .pending })
    fixture.status.clear()

    for index in 0..<5 {
      clock.set(UInt64(index + 3))
      fixture.provider.emit(sample(timestampNs: UInt64(index + 300)))
    }

    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 7 })
    XCTAssertEqual(fixture.status.count, 0)

    clock.set(102)
    fixture.provider.emit(sample(timestampNs: 800))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 8 })
    XCTAssertEqual(fixture.status.count, 1)
    XCTAssertEqual(fixture.status.latest?.lastSequence, 7)
  }

  func testTrackingTransitionPublishesImmediatelyDuringStatusThrottle() {
    let clock = ManualClock(0)
    let fixture = makeCoordinator(
      sessionIDs: [52],
      clock: clock.now,
      statusPublishIntervalNs: 100
    )
    fixture.coordinator.connect(endpoint: DatagramEndpoint(host: "127.0.0.1", port: 5005))
    XCTAssertTrue(waitUntil { fixture.transport.endpoint != nil })
    fixture.transport.transition(to: .ready)
    clock.set(1)
    fixture.provider.emit(sample(timestampNs: 100))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 1 })
    clock.set(2)
    fixture.provider.emit(sample(timestampNs: 200, tracking: .normal))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 2 })
    XCTAssertTrue(waitUntil { fixture.status.latest?.settingsState == .pending })
    fixture.status.clear()

    clock.set(3)
    fixture.provider.emit(sample(timestampNs: 300, tracking: .normal))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 3 })
    XCTAssertEqual(fixture.status.count, 0)

    fixture.provider.emit(sample(timestampNs: 400, tracking: .limited))
    XCTAssertTrue(waitUntil { fixture.transport.sentPoses.count == 4 })
    XCTAssertEqual(fixture.status.latest?.trackingQuality, .limited)
    XCTAssertEqual(fixture.status.count, 1)
  }

  private func makeCoordinator(
    sessionIDs: [UInt32],
    clock: @escaping @Sendable () -> UInt64 = { DispatchTime.now().uptimeNanoseconds },
    statusPublishIntervalNs: UInt64 = 100_000_000
  ) -> (
    coordinator: TeleopCoordinator,
    provider: ManualPoseProvider,
    transport: MockDatagramTransport,
    controls: ControlStateStore,
    status: StatusCollector
  ) {
    let provider = ManualPoseProvider()
    let transport = MockDatagramTransport()
    let controls = ControlStateStore()
    let status = StatusCollector()
    let sessions = SessionSequence(sessionIDs)
    let coordinator = TeleopCoordinator(
      provider: provider,
      transport: transport,
      controls: controls,
      clock: clock,
      sessionIDGenerator: sessions.next,
      statusPublishIntervalNs: statusPublishIntervalNs,
      statusHandler: status.append
    )
    return (coordinator, provider, transport, controls, status)
  }

  private func sample(
    timestampNs: UInt64,
    tracking: TrackingQuality = .normal
  ) -> PoseSample {
    PoseSample(
      timestampNs: timestampNs,
      pose: .identity,
      linearVelocity: nil,
      angularVelocity: nil,
      trackingQuality: tracking
    )
  }

  private func pose(from data: Data) throws -> PosePacket {
    guard case .pose(let packet) = try PacketCodec.decode(data) else {
      throw TestError.unexpectedPacket
    }
    return packet
  }

  private func ping(from data: Data) throws -> PingPacket {
    guard case .ping(let packet) = try PacketCodec.decode(data) else {
      throw TestError.unexpectedPacket
    }
    return packet
  }

  private func settings(from data: Data) throws -> SettingsPacket {
    guard case .settings(let packet) = try PacketCodec.decode(data) else {
      throw TestError.unexpectedPacket
    }
    return packet
  }

  private func estopResetRequest(from data: Data) throws -> EStopResetRequestPacket {
    guard case .estopResetRequest(let packet) = try PacketCodec.decode(data) else {
      throw TestError.unexpectedPacket
    }
    return packet
  }

  private func waitUntil(
    timeout: TimeInterval = 1,
    condition: () -> Bool
  ) -> Bool {
    let deadline = Date().addingTimeInterval(timeout)
    while Date() < deadline {
      if condition() { return true }
      Thread.sleep(forTimeInterval: 0.002)
    }
    return condition()
  }

  private enum TestError: Error {
    case unexpectedPacket
  }
}
