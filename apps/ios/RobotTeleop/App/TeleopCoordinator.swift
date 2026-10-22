import Foundation

struct PhoneControlReply: Decodable, Equatable, Sendable {
  let protocolName: String
  let id: UInt32
  let sessionID: UInt32
  let ok: Bool
  let message: String
  let kernelRuntimeID: String?
  let enabled: Bool
  let active: Bool
  let estop: Bool
  let targetLimited: Bool
  let robotGripper: Double?
  enum CodingKeys: String, CodingKey {
    case protocolName = "protocol", id, sessionID = "session_id", ok, message
    case kernelRuntimeID = "kernel_runtime_id", enabled, active, estop
    case targetLimited = "target_limited", robotGripper = "robot_gripper"
  }
}

private struct PhoneControlRequest: Encodable {
  let protocolName = "tia.phone.control.v1"
  let id: UInt32
  let sessionID: UInt32
  let clientTimeNs: UInt64
  let action: String
  let runtimeID: String?
  enum CodingKeys: String, CodingKey {
    case protocolName = "protocol", id, sessionID = "session_id", clientTimeNs = "client_time_ns"
    case action, runtimeID = "runtime_id"
  }
}

enum SettingsDeliveryState: String, Equatable, Sendable {
  case notSent
  case pending
  case acknowledged
}

enum EStopResetDeliveryState: String, Equatable, Sendable {
  case notRequested
  case pending
  case acknowledged
}

struct TeleopCoordinatorStatus: Equatable, Sendable {
  var networkState: DatagramTransportState = .disconnected
  var sessionID: UInt32 = 0
  var trackingQuality = TrackingQuality.unavailable
  var sendRateHz: Double = 0
  var lastSequence: UInt32?
  var rttMs: Double?
  var clockOffsetNs: Double?
  var settingsState = SettingsDeliveryState.notSent
  var estopResetState = EStopResetDeliveryState.notRequested
  var replacedPoseCount: UInt64 = 0
  var invalidResponseCount: UInt64 = 0
  var captureTelemetry: CaptureTelemetry?
  var trackingReason: String?
  var phoneControl: PhoneControlReply?
  var controlAvailable = false
  var controlPending = false
  var controlMessage: String?
}

final class TeleopCoordinator: @unchecked Sendable {
  private struct PendingPing {
    let packet: PingPacket
    let sentAtNs: UInt64
  }

  private struct PendingSettings {
    let packet: SettingsPacket
    let sentAtNs: UInt64
  }

  private struct PendingEStopReset {
    let packet: EStopResetRequestPacket
    var sentAtNs: UInt64
  }

  private let provider: any PoseProvider
  private let transport: any DatagramTransport
  private let controls: ControlStateStore
  private let clock: @Sendable () -> UInt64
  private let sessionIDGenerator: @Sendable () -> UInt32
  private let statusPublishIntervalNs: UInt64
  private let statusHandler: @Sendable (TeleopCoordinatorStatus) -> Void
  private let queue = DispatchQueue(
    label: "com.mo.robotteleop.coordinator",
    qos: .userInteractive
  )
  private let clockEstimator = ClockSyncEstimator()

  private var status = TeleopCoordinatorStatus()
  private var active = false
  private var networkReady = false
  private var hasSentBaselinePose = false
  private var poseSequence: UInt32 = 0
  private var pingSequence: UInt32 = 0
  private var settingsSequence: UInt32 = 0
  private var estopResetSequence: UInt32 = 0
  private var lastSample: PoseSample?
  private var pendingPing: PendingPing?
  private var pendingSettings: PendingSettings?
  private var pendingEStopReset: PendingEStopReset?
  private var timer: DispatchSourceTimer?
  private var rateWindowStartedNs: UInt64?
  private var rateWindowSamples: UInt64 = 0
  private var lastPoseTimestampNs: UInt64?
  private var lastSentClutchActive = false
  private var lastSentEstopRequested = false
  private var pingSendRequested = false
  private var settingsSendRequested = false
  private var lastStatusPublicationNs: UInt64?
  private var controlSequence: UInt32 = 0
  private var pendingControl: (id: UInt32, action: String, data: Data, since: UInt64)?
  private var lastControlSendNs: UInt64 = 0
  private var lastControlReplyNs: UInt64?

  init(
    provider: any PoseProvider,
    transport: any DatagramTransport,
    controls: ControlStateStore = ControlStateStore(),
    clock: @escaping @Sendable () -> UInt64 = { DispatchTime.now().uptimeNanoseconds },
    sessionIDGenerator: @escaping @Sendable () -> UInt32 = {
      UInt32.random(in: 1...UInt32.max)
    },
    statusPublishIntervalNs: UInt64 = 100_000_000,
    statusHandler: @escaping @Sendable (TeleopCoordinatorStatus) -> Void
  ) {
    precondition(statusPublishIntervalNs > 0)
    self.provider = provider
    self.transport = transport
    self.controls = controls
    self.clock = clock
    self.sessionIDGenerator = sessionIDGenerator
    self.statusPublishIntervalNs = statusPublishIntervalNs
    self.statusHandler = statusHandler
  }

  func connect(endpoint: DatagramEndpoint) {
    queue.async { [weak self] in
      self?.connectOnQueue(endpoint: endpoint)
    }
  }

  func disconnect() {
    queue.async { [weak self] in
      self?.disconnectOnQueue()
    }
  }

  func setClutchActive(_ isActive: Bool) {
    let changed = controls.setClutchActive(isActive)
    if changed, !isActive {
      queue.async { [weak self] in
        self?.sendSafetyEdgeOnQueue()
      }
    }
  }

  func requestRecenter() {
    controls.requestRecenter()
  }

  func requestHome() {
    controls.requestHome()
    queue.async { [weak self] in
      self?.sendHomeRequestOnQueue()
    }
  }

  func requestJointZero() {
    controls.requestJointZero()
    queue.async { [weak self] in
      self?.sendJointZeroRequestOnQueue()
    }
  }

  func latchEstop() {
    if controls.latchEstop() {
      queue.async { [weak self] in
        guard let self else { return }
        self.pendingEStopReset = nil
        self.status.estopResetState = .notRequested
        self.sendSafetyEdgeOnQueue()
      }
    }
  }

  func requestEstopReset() {
    queue.async { [weak self] in
      self?.requestEstopResetOnQueue()
    }
  }

  func setGripper(_ value: Float) {
    controls.setGripper(value)
  }

  func setMappingScales(position: Float, rotation: Float) {
    controls.setMappingScales(position: position, rotation: rotation)
    queue.async { [weak self] in
      self?.settingsSendRequested = true
    }
  }

  func requestPing() {
    queue.async { [weak self] in
      self?.requestPingOnQueue()
    }
  }

  func requestControl(_ action: String) {
    queue.async { [weak self] in
      guard let self, action == "enable" || action == "disable" else { return }
      self.controls.releaseClutch()
      self.sendSafetyEdgeOnQueue()
      self.sendControlOnQueue(action)
    }
  }

  private func sendControlOnQueue(_ action: String) {
    guard active, networkReady, hasSentBaselinePose else { return }
    let now = clock()
    let packet = PhoneControlRequest(id: controlSequence, sessionID: status.sessionID,
      clientTimeNs: now, action: action, runtimeID: status.phoneControl?.kernelRuntimeID)
    controlSequence &+= 1
    do {
      let data = try JSONEncoder().encode(packet)
      transport.sendControl(data)
      pendingControl = (packet.id, action, data, now)
      lastControlSendNs = now
      status.controlPending = action != "monitor"
      if action != "monitor" { status.controlMessage = "等待机器人确认…" }
      publishStatus(force: action != "monitor")
    } catch {
      status.controlMessage = error.localizedDescription
      publishStatus(force: true)
    }
  }

  private func controlTickOnQueue() {
    let now = clock()
    if let lastControlReplyNs, elapsed(from: lastControlReplyNs, to: now) > 1_500_000_000 {
      status.controlAvailable = false
      controls.releaseClutch()
      sendSafetyEdgeOnQueue()
    }
    if let pendingControl {
      if elapsed(from: pendingControl.since, to: now) >= 3_000_000_000 {
        self.pendingControl = nil
        status.controlPending = false
        status.controlAvailable = false
        status.controlMessage = "操作未确认，请检查电脑接收服务"
      } else if elapsed(from: lastControlSendNs, to: now) >= 500_000_000 {
        transport.sendControl(pendingControl.data)
        lastControlSendNs = now
      }
    } else if elapsed(from: lastControlSendNs, to: now) >= 250_000_000 {
      sendControlOnQueue("monitor")
    }
  }

  private func connectOnQueue(endpoint: DatagramEndpoint) {
    guard !active else { return }
    active = true
    networkReady = false
    hasSentBaselinePose = false
    poseSequence = 0
    pingSequence = 0
    settingsSequence = 0
    estopResetSequence = 0
    lastSample = nil
    pendingPing = nil
    pendingSettings = nil
    pendingEStopReset = nil
    rateWindowStartedNs = nil
    rateWindowSamples = 0
    lastPoseTimestampNs = nil
    lastSentClutchActive = false
    lastSentEstopRequested = false
    pingSendRequested = false
    settingsSendRequested = false
    pendingControl = nil
    controlSequence = 0
    lastControlSendNs = 0
    lastControlReplyNs = nil
    clockEstimator.reset()
    controls.deactivate()
    lastStatusPublicationNs = nil

    let generatedSessionID = sessionIDGenerator()
    status = TeleopCoordinatorStatus(
      networkState: .preparing,
      sessionID: generatedSessionID == 0 ? 1 : generatedSessionID
    )
    publishStatus(force: true)

    transport.start(
      endpoint: endpoint,
      receiveHandler: { [weak self] data in
        self?.queue.async { [weak self] in
          self?.handleInboundOnQueue(data)
        }
      },
      stateHandler: { [weak self] state in
        self?.queue.async { [weak self] in
          self?.handleTransportStateOnQueue(state)
        }
      }
    )
    provider.start { [weak self] sample in
      self?.queue.async { [weak self] in
        self?.handleSampleOnQueue(sample)
      }
    }
  }

  private func disconnectOnQueue() {
    guard active else {
      transport.stop(finalDatagram: nil)
      return
    }

    controls.deactivate()
    stopTimer()
    provider.stop()
    let finalDatagram = makeFinalReleasedDatagram()
    active = false
    networkReady = false
    pendingPing = nil
    pendingSettings = nil
    pendingEStopReset = nil
    pingSendRequested = false
    settingsSendRequested = false
    lastSample = nil
    status.networkState = .disconnected
    status.controlAvailable = false
    status.controlPending = false
    pendingControl = nil
    status.trackingQuality = .unavailable
    status.estopResetState = .notRequested
    publishStatus(force: true)
    transport.stop(finalDatagram: finalDatagram)
  }

  private func handleTransportStateOnQueue(_ state: DatagramTransportState) {
    status.networkState = state
    switch state {
    case .ready:
      guard active else { return }
      networkReady = true
      hasSentBaselinePose = false
      startTimer()
    case .preparing, .waiting:
      networkReady = false
      hasSentBaselinePose = false
      controls.deactivate()
      pendingEStopReset = nil
      status.estopResetState = .notRequested
    case .failed, .disconnected:
      networkReady = false
      controls.deactivate()
      stopTimer()
      if active {
        active = false
        provider.stop()
        lastSample = nil
        pendingPing = nil
        pendingSettings = nil
        pendingEStopReset = nil
        status.trackingQuality = .unavailable
        status.estopResetState = .notRequested
      }
    }
    publishStatus(force: true)
  }

  private func handleSampleOnQueue(_ sample: PoseSample) {
    guard active else { return }
    lastSample = sample
    let trackingChanged = status.trackingQuality != sample.trackingQuality
    status.trackingQuality = sample.trackingQuality
    if let captureTelemetry = sample.captureTelemetry {
      status.captureTelemetry = captureTelemetry
    }
    if sample.trackingQuality != .normal {
      controls.deactivate()
      pendingEStopReset = nil
      status.estopResetState = .notRequested
    }
    guard networkReady else {
      publishStatus(force: trackingChanged)
      return
    }

    let control = controls.snapshotForPose()
    let forceReleased = !hasSentBaselinePose
    do {
      let packet = makePosePacket(
        sample: sample,
        control: control,
        forceReleased: forceReleased,
        timestampNs: nextPoseTimestamp(preferred: sample.timestampNs)
      )
      transport.sendPose(try PacketCodec.encode(.pose(packet)))
      status.lastSequence = packet.sequence
      poseSequence &+= 1
      hasSentBaselinePose = true
      lastSentClutchActive = packet.clutchActive
      lastSentEstopRequested = packet.estopRequested
      updateSendRate()
      publishStatus(force: trackingChanged)
      if status.settingsState == .notSent {
        settingsSendRequested = true
      }
      sendRequestedControlAfterPoseOnQueue(allowControl: !forceReleased)
      status.trackingReason = sample.trackingReason
      controlTickOnQueue()
    } catch {
      status.invalidResponseCount &+= 1
      publishStatus(force: true)
    }
  }

  private func makePosePacket(
    sample: PoseSample,
    control: ControlSnapshot,
    forceReleased: Bool,
    timestampNs: UInt64
  ) -> PosePacket {
    PosePacket(
      sequence: poseSequence,
      sessionID: status.sessionID,
      clientTimeNs: timestampNs,
      pose: sample.pose,
      linearVelocity: sample.linearVelocity,
      angularVelocity: sample.angularVelocity,
      connected: networkReady,
      clutchActive: forceReleased ? false : control.clutchActive,
      recenterRequested: forceReleased ? false : control.recenterRequested,
      estopRequested: control.estopRequested,
      gripper: control.gripper,
      trackingQuality: sample.trackingQuality,
      homeRequested: forceReleased ? false : control.homeRequested,
      jointZeroRequested: forceReleased ? false : control.jointZeroRequested
    )
  }

  private func makeFinalReleasedDatagram() -> Data? {
    guard networkReady, let lastSample else { return nil }
    let control = controls.snapshotForPose()
    let nextTimestamp = nextPoseTimestamp(preferred: clock())
    let packet = makePosePacket(
      sample: lastSample,
      control: control,
      forceReleased: true,
      timestampNs: nextTimestamp
    )
    return try? PacketCodec.encode(.pose(packet))
  }

  private func sendSafetyEdgeOnQueue() {
    guard active, networkReady, hasSentBaselinePose, let lastSample else { return }
    let control = controls.snapshot
    guard lastSentClutchActive || (control.estopRequested && !lastSentEstopRequested) else {
      return
    }
    sendForcedReleasedPoseOnQueue(
      sample: lastSample,
      control: control,
      preferredTimestamp: clock()
    )
  }

  private func sendForcedReleasedPoseOnQueue(
    sample: PoseSample,
    control: ControlSnapshot,
    preferredTimestamp: UInt64
  ) {
    let packet = makePosePacket(
      sample: sample,
      control: control,
      forceReleased: true,
      timestampNs: nextPoseTimestamp(preferred: preferredTimestamp)
    )
    do {
      transport.sendPose(try PacketCodec.encode(.pose(packet)))
      status.lastSequence = packet.sequence
      poseSequence &+= 1
      lastSentClutchActive = false
      lastSentEstopRequested = packet.estopRequested
      updateSendRate()
      publishStatus(force: true)
    } catch {
      status.invalidResponseCount &+= 1
      publishStatus(force: true)
    }
  }

  private func sendHomeRequestOnQueue() {
    guard active, networkReady, hasSentBaselinePose, let lastSample else { return }
    let control = controls.snapshotForPose()
    guard control.homeRequested else { return }
    let packet = makePosePacket(
      sample: lastSample,
      control: control,
      forceReleased: false,
      timestampNs: nextPoseTimestamp(preferred: clock())
    )
    do {
      transport.sendPose(try PacketCodec.encode(.pose(packet)))
      status.lastSequence = packet.sequence
      poseSequence &+= 1
      lastSentClutchActive = false
      lastSentEstopRequested = packet.estopRequested
      updateSendRate()
      publishStatus(force: true)
    } catch {
      status.invalidResponseCount &+= 1
      publishStatus(force: true)
    }
  }

  private func sendJointZeroRequestOnQueue() {
    guard active, networkReady, hasSentBaselinePose, let lastSample else { return }
    let control = controls.snapshotForPose()
    guard control.jointZeroRequested else { return }
    let packet = makePosePacket(
      sample: lastSample,
      control: control,
      forceReleased: false,
      timestampNs: nextPoseTimestamp(preferred: clock())
    )
    do {
      transport.sendPose(try PacketCodec.encode(.pose(packet)))
      status.lastSequence = packet.sequence
      poseSequence &+= 1
      lastSentClutchActive = false
      lastSentEstopRequested = packet.estopRequested
      updateSendRate()
      publishStatus(force: true)
    } catch {
      status.invalidResponseCount &+= 1
      publishStatus(force: true)
    }
  }

  private func requestEstopResetOnQueue() {
    guard active, networkReady, hasSentBaselinePose, let lastSample else { return }
    guard lastSample.trackingQuality == .normal, controls.beginEstopReset() else { return }

    let now = clock()
    let request = EStopResetRequestPacket(
      sequence: estopResetSequence,
      sessionID: status.sessionID,
      clientTimeNs: now
    )
    estopResetSequence &+= 1
    do {
      transport.sendControl(try PacketCodec.encode(.estopResetRequest(request)))
      pendingEStopReset = PendingEStopReset(packet: request, sentAtNs: now)
      status.estopResetState = .pending
      let releaseTimestamp = now == UInt64.max ? now : now + 1
      sendForcedReleasedPoseOnQueue(
        sample: lastSample,
        control: controls.snapshot,
        preferredTimestamp: releaseTimestamp
      )
    } catch {
      controls.cancelPendingEstopReset()
      status.estopResetState = .notRequested
      status.invalidResponseCount &+= 1
      publishStatus(force: true)
    }
  }

  private func nextPoseTimestamp(preferred: UInt64) -> UInt64 {
    guard let previous = lastPoseTimestampNs else {
      lastPoseTimestampNs = preferred
      return preferred
    }
    let timestamp = previous == UInt64.max ? UInt64.max : max(preferred, previous + 1)
    lastPoseTimestampNs = timestamp
    return timestamp
  }

  private func requestPingOnQueue() {
    guard active, networkReady else { return }
    let now = clock()
    if let pendingPing {
      guard elapsed(from: pendingPing.sentAtNs, to: now) >= 3_000_000_000 else {
        return
      }
      self.pendingPing = nil
    }
    pingSendRequested = true
  }

  private func sendPingOnQueue() {
    let now = clock()
    let packet = PingPacket(
      sequence: pingSequence,
      sessionID: status.sessionID,
      clientTimeNs: now
    )
    pingSequence &+= 1
    do {
      transport.sendControl(try PacketCodec.encode(.ping(packet)))
      pendingPing = PendingPing(packet: packet, sentAtNs: now)
    } catch {
      status.invalidResponseCount &+= 1
      publishStatus(force: true)
    }
  }

  private func sendSettingsOnQueue() {
    guard active, networkReady, hasSentBaselinePose else { return }
    let control = controls.snapshot
    let now = clock()
    let packet = SettingsPacket(
      sequence: settingsSequence,
      sessionID: status.sessionID,
      clientTimeNs: now,
      positionScale: control.positionScale,
      rotationScale: control.rotationScale
    )
    settingsSequence &+= 1
    do {
      transport.sendControl(try PacketCodec.encode(.settings(packet)))
      pendingSettings = PendingSettings(packet: packet, sentAtNs: now)
      status.settingsState = .pending
      publishStatus(force: true)
    } catch {
      status.invalidResponseCount &+= 1
      publishStatus(force: true)
    }
  }

  private func sendRequestedControlAfterPoseOnQueue(allowControl: Bool) {
    guard allowControl else { return }
    if settingsSendRequested {
      settingsSendRequested = false
      sendSettingsOnQueue()
      return
    }
    if pingSendRequested {
      pingSendRequested = false
      sendPingOnQueue()
    }
  }

  private func handleInboundOnQueue(_ data: Data) {
    guard active else { return }
    if data.first == 123 {
      guard data.count <= 2048,
        let reply = try? JSONDecoder().decode(PhoneControlReply.self, from: data),
        reply.protocolName == "tia.phone.control.v1",
        reply.sessionID == status.sessionID,
        let pendingControl, reply.id == pendingControl.id
      else { rejectResponse(); return }
      self.pendingControl = nil
      lastControlReplyNs = clock()
      if !reply.message.isEmpty || pendingControl.action != "monitor" || !status.controlAvailable {
        status.controlMessage = reply.message.isEmpty ? nil : reply.message
      }
      status.phoneControl = reply
      status.controlAvailable = true
      status.controlPending = false
      if !reply.enabled || reply.estop {
        controls.releaseClutch()
        sendSafetyEdgeOnQueue()
      }
      publishStatus(force: true)
      return
    }
    do {
      switch try PacketCodec.decode(data) {
      case .pong(let packet):
        handlePongOnQueue(packet)
      case .settingsAck(let packet):
        handleSettingsAckOnQueue(packet)
      case .estopResetAck(let packet):
        handleEstopResetAckOnQueue(packet)
      case .pose, .ping, .settings, .estopResetRequest:
        rejectResponse()
      }
    } catch {
      rejectResponse()
    }
  }

  private func handlePongOnQueue(_ packet: PongPacket) {
    guard let pendingPing,
      packet.sessionID == status.sessionID,
      packet.sequence == pendingPing.packet.sequence,
      packet.clientTimeNs == pendingPing.packet.clientTimeNs
    else {
      rejectResponse()
      return
    }
    self.pendingPing = nil
    do {
      let sample = try ClockSample.calculate(
        clientSendTimeNs: packet.clientTimeNs,
        serverReceiveTimeNs: packet.serverReceiveTimeNs,
        serverSendTimeNs: packet.serverSendTimeNs,
        clientReceiveTimeNs: clock()
      )
      guard clockEstimator.add(sample) else {
        rejectResponse()
        return
      }
      status.rttMs = Double(sample.rttNs) / 1_000_000
      status.clockOffsetNs = clockEstimator.offsetNs
      publishStatus(force: true)
    } catch {
      rejectResponse()
    }
  }

  private func handleSettingsAckOnQueue(_ packet: SettingsAckPacket) {
    guard let pendingSettings,
      packet.sessionID == status.sessionID,
      packet.sequence == pendingSettings.packet.sequence,
      packet.clientTimeNs == pendingSettings.packet.clientTimeNs,
      packet.positionScale == pendingSettings.packet.positionScale,
      packet.rotationScale == pendingSettings.packet.rotationScale
    else {
      rejectResponse()
      return
    }
    self.pendingSettings = nil
    status.settingsState = .acknowledged
    publishStatus(force: true)
  }

  private func handleEstopResetAckOnQueue(_ packet: EStopResetAckPacket) {
    guard let pendingEStopReset,
      packet.sourceType == pendingEStopReset.packet.sourceType,
      packet.sessionID == status.sessionID,
      packet.sequence == pendingEStopReset.packet.sequence,
      packet.clientTimeNs == pendingEStopReset.packet.clientTimeNs,
      controls.completeEstopReset()
    else {
      rejectResponse()
      return
    }
    self.pendingEStopReset = nil
    status.estopResetState = .acknowledged
    status.controlMessage = nil
    publishStatus(force: true)
  }

  private func rejectResponse() {
    status.invalidResponseCount &+= 1
    publishStatus(force: true)
  }

  private func startTimer() {
    guard timer == nil else { return }
    let timer = DispatchSource.makeTimerSource(queue: queue)
    self.timer = timer
    timer.schedule(
      deadline: .now() + .seconds(1),
      repeating: .seconds(1),
      leeway: .milliseconds(50)
    )
    timer.setEventHandler { [weak self] in
      self?.timerTickOnQueue()
    }
    timer.activate()
  }

  private func stopTimer() {
    let timer = timer
    self.timer = nil
    timer?.cancel()
  }

  private func timerTickOnQueue() {
    controlTickOnQueue()
    requestPingOnQueue()
    let now = clock()
    if let pendingSettings,
      elapsed(from: pendingSettings.sentAtNs, to: now) >= 1_000_000_000
    {
      settingsSendRequested = true
    }
    guard var pendingEStopReset else { return }
    if elapsed(from: pendingEStopReset.packet.clientTimeNs, to: now) >= 5_000_000_000 {
      self.pendingEStopReset = nil
      controls.cancelPendingEstopReset()
      status.estopResetState = .notRequested
      status.controlMessage = "恢复未确认；检查追踪和连接后再试"
      publishStatus(force: true)
      return
    }
    guard elapsed(from: pendingEStopReset.sentAtNs, to: now) >= 1_000_000_000 else {
      return
    }
    do {
      transport.sendControl(try PacketCodec.encode(.estopResetRequest(pendingEStopReset.packet)))
      pendingEStopReset.sentAtNs = now
      self.pendingEStopReset = pendingEStopReset
    } catch {
      status.invalidResponseCount &+= 1
      publishStatus(force: true)
    }
  }

  private func updateSendRate() {
    let now = clock()
    rateWindowSamples &+= 1
    guard let startedAt = rateWindowStartedNs else {
      rateWindowStartedNs = now
      return
    }
    let duration = elapsed(from: startedAt, to: now)
    guard duration >= 1_000_000_000 else { return }
    status.sendRateHz = Double(rateWindowSamples) * 1_000_000_000 / Double(duration)
    rateWindowStartedNs = now
    rateWindowSamples = 0
  }

  private func elapsed(from start: UInt64, to end: UInt64) -> UInt64 {
    end >= start ? end - start : 0
  }

  private func publishStatus(force: Bool = false) {
    status.replacedPoseCount = transport.replacedPoseCount
    let now = clock()
    guard
      force
        || lastStatusPublicationNs == nil
        || elapsed(from: lastStatusPublicationNs ?? now, to: now) >= statusPublishIntervalNs
    else {
      return
    }
    lastStatusPublicationNs = now
    statusHandler(status)
  }
}
