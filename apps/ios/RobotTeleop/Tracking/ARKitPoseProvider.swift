@preconcurrency import ARKit
import Foundation
import UIKit
import simd

struct ARKitCaptureFormatCandidate: Equatable {
  let framesPerSecond: Int
  let width: Int
  let height: Int

  fileprivate var pixelCount: Int64 {
    Int64(width) * Int64(height)
  }
}

enum ARKitCaptureFormatSelector {
  static func preferredIndex(in candidates: [ARKitCaptureFormatCandidate]) -> Int? {
    candidates.indices.min { leftIndex, rightIndex in
      let left = candidates[leftIndex]
      let right = candidates[rightIndex]
      if left.framesPerSecond != right.framesPerSecond {
        return left.framesPerSecond > right.framesPerSecond
      }
      if left.pixelCount != right.pixelCount {
        return left.pixelCount < right.pixelCount
      }
      return leftIndex < rightIndex
    }
  }
}

/// The pose contract shared by the simulator and a physical phone.
///
/// ARKit reports the camera in an arbitrary world frame.  Teleoperation only
/// needs a stable, local phone frame: X=phone right, Y=phone top, Z=screen
/// normal.  The camera axes use that same handedness, so the only conversion
/// required here is rebasing the first valid camera transform to identity.
enum ARKitPoseFrame {
  static func localTransform(
    reference: simd_float4x4,
    current: simd_float4x4
  ) -> simd_float4x4 {
    simd_inverse(reference) * current
  }

  static func pose(
    reference: simd_float4x4,
    current: simd_float4x4
  ) -> TeleopPose {
    let transform = localTransform(reference: reference, current: current)
    let rawQuaternion = simd_quatf(transform)
    let norm = simd_length(rawQuaternion.vector)
    guard norm.isFinite, norm > 0 else { return .identity }
    let quaternion = rawQuaternion.vector / norm
    return TeleopPose(
      position: TeleopVector3(
        x: transform.columns.3.x,
        y: transform.columns.3.y,
        z: transform.columns.3.z
      ),
      orientation: TeleopQuaternion(
        x: quaternion.x,
        y: quaternion.y,
        z: quaternion.z,
        w: quaternion.w
      )
    )
  }
}

struct FrameCadenceMeter {
  private var windowStartedNs: UInt64?
  private var intervalCount: UInt64 = 0

  mutating func reset() {
    windowStartedNs = nil
    intervalCount = 0
  }

  mutating func record(timestampNs: UInt64) -> Double? {
    guard let startedAtNs = windowStartedNs else {
      windowStartedNs = timestampNs
      return nil
    }
    if timestampNs < startedAtNs {
      windowStartedNs = timestampNs
      intervalCount = 0
      return nil
    }
    guard timestampNs > startedAtNs else { return nil }

    intervalCount &+= 1
    let elapsedNs = timestampNs - startedAtNs
    guard elapsedNs >= 1_000_000_000 else { return nil }

    let framesPerSecond = Double(intervalCount) * 1_000_000_000 / Double(elapsedNs)
    windowStartedNs = timestampNs
    intervalCount = 0
    return framesPerSecond
  }
}

final class ARKitPoseProvider: NSObject, PoseProvider, ARSessionDelegate, @unchecked Sendable {
  let kind = PoseProviderKind.arkit

  private let session = ARSession()
  private let queue = DispatchQueue(
    label: "com.mo.robotteleop.tracking.arkit",
    qos: .userInteractive
  )
  private let lock = NSLock()
  private var handler: (@Sendable (PoseSample) -> Void)?
  private var lastPose = TeleopPose.identity
  private var running = false
  private var referenceTransform: simd_float4x4?
  private var captureTelemetry: CaptureTelemetry?
  private var captureCadenceMeter = FrameCadenceMeter()

  override init() {
    super.init()
    session.delegate = self
    session.delegateQueue = queue
  }

  func start(handler: @escaping @Sendable (PoseSample) -> Void) {
    lock.lock()
    guard !running else {
      lock.unlock()
      return
    }
    running = true
    self.handler = handler
    lock.unlock()

    queue.async { [session] in
      guard ARWorldTrackingConfiguration.isSupported else {
        return
      }
      let configuration = Self.makeConfiguration()
      self.referenceTransform = nil
      self.captureTelemetry = Self.captureTelemetry(for: configuration.videoFormat)
      self.captureCadenceMeter.reset()
      session.run(
        configuration,
        options: [.resetTracking, .removeExistingAnchors]
      )
    }
  }

  func stop() {
    lock.lock()
    running = false
    handler = nil
    lock.unlock()
    queue.async { [session] in
      self.referenceTransform = nil
      session.pause()
    }
  }

  func session(_ session: ARSession, didUpdate frame: ARFrame) {
    // ARCamera.transform uses the sensor-native camera frame. The app's
    // canonical phone frame follows the portrait display axes, so ask ARKit
    // for the interface-oriented view matrix before rebasing it.
    let transform = simd_inverse(frame.camera.viewMatrix(for: .portrait))
    let reference =
      referenceTransform
      ?? {
        referenceTransform = transform
        return transform
      }()
    let pose = ARKitPoseFrame.pose(reference: reference, current: transform)
    let timestampNs =
      frame.timestamp > 0
      ? UInt64(frame.timestamp * 1_000_000_000)
      : monotonicNanoseconds()
    let telemetry = updateCaptureTelemetry(timestampNs: timestampNs)
    publish(
      PoseSample(
        timestampNs: timestampNs,
        pose: pose,
        linearVelocity: nil,
        angularVelocity: nil,
        trackingQuality: trackingQuality(frame.camera.trackingState),
        captureTelemetry: telemetry,
        trackingReason: trackingReason(frame.camera.trackingState)
      )
    )
  }

  func sessionWasInterrupted(_ session: ARSession) {
    publishUnavailable(timestampNs: monotonicNanoseconds())
  }

  func session(_ session: ARSession, didFailWithError error: any Error) {
    publishUnavailable(timestampNs: monotonicNanoseconds())
  }

  private func trackingQuality(_ state: ARCamera.TrackingState) -> TrackingQuality {
    switch state {
    case .normal:
      .normal
    case .limited:
      .limited
    case .notAvailable:
      .unavailable
    }
  }

  private func trackingReason(_ state: ARCamera.TrackingState) -> String? {
    switch state {
    case .normal: return nil
    case .notAvailable: return "追踪不可用，请检查相机"
    case .limited(let reason):
      switch reason {
      case .initializing: return "正在初始化，缓慢观察周围环境"
      case .excessiveMotion: return "手机移动过快或画面模糊"
      case .insufficientFeatures: return "画面特征不足，请对准有纹理、光线充足的区域"
      case .relocalizing: return "正在重新定位，请观察之前的环境"
      @unknown default: return "追踪受限，请保持画面清晰"
      }
    }
  }

  private func publish(_ sample: PoseSample) {
    lock.lock()
    guard running, let handler else {
      lock.unlock()
      return
    }
    lastPose = sample.pose
    lock.unlock()
    handler(sample)
  }

  private func publishUnavailable(timestampNs: UInt64) {
    lock.lock()
    let pose = lastPose
    lock.unlock()
    publish(
      PoseSample(
        timestampNs: timestampNs,
        pose: pose,
        linearVelocity: nil,
        angularVelocity: nil,
        trackingQuality: .unavailable,
        captureTelemetry: captureTelemetry
      )
    )
  }

  private func monotonicNanoseconds() -> UInt64 {
    DispatchTime.now().uptimeNanoseconds
  }

  static func makeConfiguration() -> ARWorldTrackingConfiguration {
    let configuration = ARWorldTrackingConfiguration()
    configuration.planeDetection = []
    configuration.environmentTexturing = .none
    configuration.wantsHDREnvironmentTextures = false
    configuration.isLightEstimationEnabled = false
    configuration.providesAudioData = false
    configuration.frameSemantics = []
    configuration.videoHDRAllowed = false

    let formats = ARWorldTrackingConfiguration.supportedVideoFormats
    let candidates = formats.map {
      ARKitCaptureFormatCandidate(
        framesPerSecond: $0.framesPerSecond,
        width: Int($0.imageResolution.width.rounded()),
        height: Int($0.imageResolution.height.rounded())
      )
    }
    if let index = ARKitCaptureFormatSelector.preferredIndex(in: candidates) {
      configuration.videoFormat = formats[index]
    }
    return configuration
  }

  private static func captureTelemetry(
    for format: ARConfiguration.VideoFormat
  ) -> CaptureTelemetry {
    CaptureTelemetry(
      configuredFramesPerSecond: format.framesPerSecond,
      configuredWidth: Int(format.imageResolution.width.rounded()),
      configuredHeight: Int(format.imageResolution.height.rounded()),
      measuredFramesPerSecond: nil
    )
  }

  private func updateCaptureTelemetry(timestampNs: UInt64) -> CaptureTelemetry? {
    guard let telemetry = captureTelemetry else { return nil }
    guard let measuredFramesPerSecond = captureCadenceMeter.record(timestampNs: timestampNs) else {
      return telemetry
    }
    let updatedTelemetry = CaptureTelemetry(
      configuredFramesPerSecond: telemetry.configuredFramesPerSecond,
      configuredWidth: telemetry.configuredWidth,
      configuredHeight: telemetry.configuredHeight,
      measuredFramesPerSecond: measuredFramesPerSecond
    )
    captureTelemetry = updatedTelemetry
    return updatedTelemetry
  }
}
