import Combine
import Foundation
import UIKit

@MainActor
final class AppModel: ObservableObject {
  @Published var host = "192.168.5.8"
  @Published var port = "5005"
  @Published private(set) var status = TeleopCoordinatorStatus()
  @Published private(set) var providerKind: PoseProviderKind
  @Published private(set) var clutchActive = false
  @Published private(set) var estopLatched = false
  @Published private(set) var endpointMessage: String?
  @Published private(set) var gripper = 0.0
  @Published private(set) var positionScale = Double(ControlDefaults.positionScale)
  @Published private(set) var rotationScale = Double(ControlDefaults.rotationScale)

  private var coordinator: TeleopCoordinator?
  private var wasNetworkReady = false

  init(
    provider: (any PoseProvider)? = nil,
    transport: (any DatagramTransport)? = nil,
    controls: ControlStateStore = ControlStateStore()
  ) {
    let launchEnvironment = ProcessInfo.processInfo.environment
    host = launchEnvironment["ROBOT_TELEOP_HOST"] ?? "192.168.5.8"
    port = launchEnvironment["ROBOT_TELEOP_PORT"] ?? "5005"

    let provider = provider ?? PoseProviderFactory.makeDefault()
    providerKind = provider.kind
    coordinator = TeleopCoordinator(
      provider: provider,
      transport: transport ?? UDPClient(),
      controls: controls
    ) { [weak self] status in
      Task { @MainActor [weak self] in
        self?.consume(status)
      }
    }
    connect()
  }

  var connectionOpen: Bool {
    switch status.networkState {
    case .preparing, .ready, .waiting:
      true
    case .disconnected, .failed:
      false
    }
  }

  var endpointEditable: Bool {
    !connectionOpen
  }

  private var canOperate: Bool {
    status.networkState == .ready
      && status.trackingQuality == .normal
      && status.controlAvailable && status.phoneControl?.enabled == true
      && !stopLatched
      && !estopResetPending
      && !status.controlPending && !presetBusy
  }

  var canControl: Bool { canOperate && status.phoneControl?.aligned == true }

  var canAlign: Bool {
    status.networkState == .ready && status.controlAvailable && status.phoneControl?.kernelRuntimeID != nil
      && status.trackingQuality == .normal && !stopLatched && !estopResetPending
      && !status.controlPending && !clutchActive && !presetBusy
  }

  var presetBusy: Bool { status.phoneControl?.preset?.busy == true }

  var stopLatched: Bool { estopLatched || status.phoneControl?.estop == true }

  var canStartControl: Bool {
    status.networkState == .ready && status.controlAvailable && status.phoneControl?.kernelRuntimeID != nil
      && status.trackingQuality == .normal && !stopLatched
      && !status.controlPending && !estopResetPending && !clutchActive
      && status.phoneControl?.enabled != true
  }

  var controlSummary: String {
    if let message = status.controlMessage, !message.isEmpty { return message }
    if !status.controlAvailable { return "等待电脑控制服务确认" }
    if stopLatched { return "停止已锁定 · 点恢复后可重新开始" }
    if status.phoneControl?.kernelRuntimeID == nil { return "robot 未运行，请在电脑总控启动" }
    if let preset = status.phoneControl?.preset, preset.busy { return preset.message }
    if status.phoneControl?.aligned != true {
      return "手机前端对准夹爪方向，点一次“对齐方向”"
    }
    if status.phoneControl?.targetLimited == true { return "目标跟随误差较大，超出部分已丢弃" }
    if let preset = status.phoneControl?.preset { return preset.message }
    return status.phoneControl?.enabled == true ? "方向已对齐 · 按住跟随，松手保持" : "方向已对齐 · 点开始遥操"
  }

  func startControl() {
    guard canStartControl else { return }
    setClutchActive(false)
    coordinator?.requestControl("enable")
  }

  func stopControl() {
    setClutchActive(false)
    coordinator?.requestControl("disable")
  }

  var canHome: Bool {
    canOperate && !clutchActive
  }

  var canRecoverControl: Bool {
    !estopResetPending && !status.controlPending
  }

  var estopResetPending: Bool {
    status.estopResetState == .pending
  }

  var recoverControlLabel: String {
    estopResetPending ? "恢复中" : "恢复"
  }

  var tuningEnabled: Bool {
    !clutchActive && !stopLatched && !estopResetPending
  }

  var captureSummary: String? {
    guard let capture = status.captureTelemetry else { return nil }
    let configured =
      "\(capture.configuredFramesPerSecond) Hz \(capture.configuredWidth)x\(capture.configuredHeight)"
    guard let measured = capture.measuredFramesPerSecond else {
      return "ARKit capture: \(configured)"
    }
    return "ARKit capture: \(configured), measured \(String(format: "%.1f", measured)) FPS"
  }

  func performConnectionAction() {
    if connectionOpen {
      disconnect()
    } else {
      connect()
    }
  }

  func connect() {
    let cleanHost = host.trimmingCharacters(in: .whitespacesAndNewlines)
    guard !cleanHost.isEmpty, let parsedPort = UInt16(port), parsedPort > 0 else {
      endpointMessage = "Enter a valid host and UDP port."
      UINotificationFeedbackGenerator().notificationOccurred(.error)
      return
    }
    endpointMessage = nil
    coordinator?.connect(endpoint: DatagramEndpoint(host: cleanHost, port: parsedPort))
  }

  func disconnect() {
    setClutchActive(false)
    coordinator?.requestControl("disable")
    coordinator?.disconnect()
  }

  func stop() {
    disconnect()
  }

  func sceneDidBecomeInactive() {
    setClutchActive(false)
  }

  func sceneDidEnterBackground() {
    disconnect()
  }

  func setClutchActive(_ active: Bool) {
    let newValue = active && canControl
    guard clutchActive != newValue else { return }
    clutchActive = newValue
    coordinator?.setClutchActive(newValue)
    UIImpactFeedbackGenerator(style: newValue ? .light : .soft).impactOccurred()
  }

  func requestHome() {
    guard canHome else { return }
    setClutchActive(false)
    coordinator?.requestControl("home")
    UIImpactFeedbackGenerator(style: .medium).impactOccurred()
  }

  func requestPregrasp() {
    guard canHome else { return }
    setClutchActive(false)
    coordinator?.requestControl("pregrasp")
    UIImpactFeedbackGenerator(style: .medium).impactOccurred()
  }

  func requestAlignment() {
    guard canAlign else { return }
    setClutchActive(false)
    coordinator?.requestControl("align")
    UIImpactFeedbackGenerator(style: .medium).impactOccurred()
  }

  func latchEstop() {
    guard !estopLatched else { return }
    estopLatched = true
    clutchActive = false
    coordinator?.latchEstop()
    UINotificationFeedbackGenerator().notificationOccurred(.error)
  }

  func recoverControl() {
    guard canRecoverControl else { return }
    setClutchActive(false)
    if !status.controlAvailable || status.networkState != .ready {
      disconnect()
      connect()
      return
    }
    coordinator?.requestEstopReset()
    UIImpactFeedbackGenerator(style: .medium).impactOccurred()
  }

  func setGripper(_ value: Double) {
    guard canControl else { return }
    gripper = min(max(value, 0), 1)
    coordinator?.setGripper(Float(gripper))
  }

  func setPositionScale(_ value: Double) {
    guard tuningEnabled else { return }
    positionScale = min(max(value, 0), Double(ControlDefaults.maximumPositionScale))
  }

  func setRotationScale(_ value: Double) {
    guard tuningEnabled else { return }
    rotationScale = min(max(value, 0), Double(ControlDefaults.maximumRotationScale))
  }

  func commitMappingScales() {
    guard tuningEnabled else { return }
    coordinator?.setMappingScales(
      position: Float(positionScale),
      rotation: Float(rotationScale)
    )
  }

  private func consume(_ newStatus: TeleopCoordinatorStatus) {
    let resetCompleted = newStatus.estopResetState == .acknowledged && status.estopResetState != .acknowledged
    let isNetworkReady = newStatus.networkState == .ready
    if isNetworkReady, !wasNetworkReady {
      UINotificationFeedbackGenerator().notificationOccurred(.success)
    }
    wasNetworkReady = isNetworkReady
    status = newStatus

    if resetCompleted {
      let wasLatched = estopLatched
      estopLatched = false
      clutchActive = false
      if wasLatched {
        UINotificationFeedbackGenerator().notificationOccurred(.success)
      }
    }

    if !canControl {
      clutchActive = false
      coordinator?.setClutchActive(false)
    }
    if !clutchActive, newStatus.controlAvailable, newStatus.phoneControl?.enabled == true,
      let actual = newStatus.phoneControl?.robotGripper {
      gripper = min(max(actual, 0), 1)
      coordinator?.setGripper(Float(gripper))
    }
    switch newStatus.networkState {
    case .failed(let message), .waiting(let message):
      endpointMessage = message
    case .ready:
      endpointMessage = nil
    case .disconnected, .preparing:
      break
    }
  }
}
