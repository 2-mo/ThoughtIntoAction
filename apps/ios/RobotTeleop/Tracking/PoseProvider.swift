import Foundation

struct TeleopVector3: Equatable, Sendable {
  let x: Float
  let y: Float
  let z: Float

  static let zero = TeleopVector3(x: 0, y: 0, z: 0)
}

struct TeleopQuaternion: Equatable, Sendable {
  let x: Float
  let y: Float
  let z: Float
  let w: Float

  static let identity = TeleopQuaternion(x: 0, y: 0, z: 0, w: 1)
}

struct TeleopPose: Equatable, Sendable {
  let position: TeleopVector3
  let orientation: TeleopQuaternion

  static let identity = TeleopPose(
    position: .zero,
    orientation: .identity
  )
}

enum TrackingQuality: UInt8, Equatable, Sendable {
  case unavailable = 0
  case limited = 1
  case normal = 2
}

struct CaptureTelemetry: Equatable, Sendable {
  let configuredFramesPerSecond: Int
  let configuredWidth: Int
  let configuredHeight: Int
  let measuredFramesPerSecond: Double?
}

struct PoseSample: Equatable, Sendable {
  let timestampNs: UInt64
  let pose: TeleopPose
  let linearVelocity: TeleopVector3?
  let angularVelocity: TeleopVector3?
  let trackingQuality: TrackingQuality
  let captureTelemetry: CaptureTelemetry?

  init(
    timestampNs: UInt64,
    pose: TeleopPose,
    linearVelocity: TeleopVector3?,
    angularVelocity: TeleopVector3?,
    trackingQuality: TrackingQuality,
    captureTelemetry: CaptureTelemetry? = nil
  ) {
    self.timestampNs = timestampNs
    self.pose = pose
    self.linearVelocity = linearVelocity
    self.angularVelocity = angularVelocity
    self.trackingQuality = trackingQuality
    self.captureTelemetry = captureTelemetry
  }
}

enum PoseProviderKind: String, Equatable, Sendable {
  case arkit
  case simulated
}

protocol PoseProvider: AnyObject, Sendable {
  var kind: PoseProviderKind { get }

  func start(handler: @escaping @Sendable (PoseSample) -> Void)
  func stop()
}

enum PoseProviderFactory {
  static func makeDefault() -> any PoseProvider {
    #if targetEnvironment(simulator)
      SimulatedPoseProvider(
        sampleRateHz: SimulatedPoseProvider.resolveSampleRateHz(
          environment: ProcessInfo.processInfo.environment
        )
      )
    #else
      ARKitPoseProvider()
    #endif
  }
}
