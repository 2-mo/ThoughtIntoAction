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

  var canControl: Bool {
    status.networkState == .ready
      && status.trackingQuality == .normal
      && !estopLatched
      && !estopResetPending
  }

  var canHome: Bool {
    status.networkState == .ready
      && status.trackingQuality == .normal
      && !estopLatched
      && !estopResetPending
  }

  var canRecoverControl: Bool {
    status.networkState == .ready
      && status.trackingQuality == .normal
      && status.estopResetState != .pending
  }

  var estopResetPending: Bool {
    status.estopResetState == .pending
  }

  var recoverControlLabel: String {
    estopResetPending ? "RECOVERING" : "RECOVER"
  }

  var tuningEnabled: Bool {
    !clutchActive && !estopLatched && !estopResetPending
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
    clutchActive = false
    coordinator?.requestHome()
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
    coordinator?.requestEstopReset()
    UIImpactFeedbackGenerator(style: .medium).impactOccurred()
  }

  func setGripper(_ value: Double) {
    guard !estopLatched else { return }
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
    let isNetworkReady = newStatus.networkState == .ready
    if isNetworkReady, !wasNetworkReady {
      UINotificationFeedbackGenerator().notificationOccurred(.success)
    }
    wasNetworkReady = isNetworkReady
    status = newStatus

    if newStatus.estopResetState == .acknowledged {
      let wasLatched = estopLatched
      estopLatched = false
      clutchActive = false
      if wasLatched {
        UINotificationFeedbackGenerator().notificationOccurred(.success)
      }
    }

    if !isNetworkReady || newStatus.trackingQuality != .normal {
      clutchActive = false
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
