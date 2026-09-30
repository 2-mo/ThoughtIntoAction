import Foundation
import XCTest
import simd

@testable import RobotTeleop

private final class SampleCollector: @unchecked Sendable {
  private let lock = NSLock()
  private var storage: [PoseSample] = []

  func append(_ sample: PoseSample) {
    lock.lock()
    storage.append(sample)
    lock.unlock()
  }

  var samples: [PoseSample] {
    lock.lock()
    defer { lock.unlock() }
    return storage
  }
}

final class ProviderTests: XCTestCase {
  func testSimulatorSampleRateUsesValidEnvironmentOverride() {
    XCTAssertEqual(
      SimulatedPoseProvider.resolveSampleRateHz(
        environment: ["ROBOT_TELEOP_SIMULATED_RATE_HZ": "120"]
      ),
      120
    )
  }

  func testSimulatorSampleRateFallsBackForInvalidEnvironmentOverride() {
    XCTAssertEqual(
      SimulatedPoseProvider.resolveSampleRateHz(
        environment: ["ROBOT_TELEOP_SIMULATED_RATE_HZ": "not-a-number"]
      ),
      60
    )
    XCTAssertEqual(
      SimulatedPoseProvider.resolveSampleRateHz(
        environment: ["ROBOT_TELEOP_SIMULATED_RATE_HZ": "0"]
      ),
      60
    )
  }

  func testCaptureFormatSelectionPrefersHighestCadenceThenLowestPixels() {
    let candidates = [
      ARKitCaptureFormatCandidate(framesPerSecond: 60, width: 1920, height: 1080),
      ARKitCaptureFormatCandidate(framesPerSecond: 120, width: 1920, height: 1080),
      ARKitCaptureFormatCandidate(framesPerSecond: 120, width: 1280, height: 720),
      ARKitCaptureFormatCandidate(framesPerSecond: 60, width: 640, height: 480),
    ]

    XCTAssertEqual(
      ARKitCaptureFormatSelector.preferredIndex(in: candidates),
      2
    )
  }

  func testCaptureFormatSelectionUsesStableOrderForTies() {
    let candidates = [
      ARKitCaptureFormatCandidate(framesPerSecond: 120, width: 1280, height: 720),
      ARKitCaptureFormatCandidate(framesPerSecond: 120, width: 1280, height: 720),
    ]

    XCTAssertEqual(
      ARKitCaptureFormatSelector.preferredIndex(in: candidates),
      0
    )
  }

  func testCaptureCadenceUsesFrameIntervalsInsteadOfFrameCount() {
    var meter = FrameCadenceMeter()

    XCTAssertNil(meter.record(timestampNs: 0))
    XCTAssertNil(meter.record(timestampNs: 500_000_000))
    XCTAssertEqual(meter.record(timestampNs: 1_000_000_000), 2)
    XCTAssertNil(meter.record(timestampNs: 1_500_000_000))
    XCTAssertEqual(meter.record(timestampNs: 2_000_000_000), 2)
  }

  func testCaptureCadenceResetsOnNonMonotonicTimestamp() {
    var meter = FrameCadenceMeter()

    XCTAssertNil(meter.record(timestampNs: 1_000_000_000))
    XCTAssertNil(meter.record(timestampNs: 900_000_000))
    XCTAssertNil(meter.record(timestampNs: 1_400_000_000))
    XCTAssertEqual(meter.record(timestampNs: 1_900_000_000), 2)
  }

  func testARKitPoseFrameRebasesWorldTranslation() {
    var reference = matrix_identity_float4x4
    reference.columns.3 = SIMD4<Float>(3, -2, 1, 1)
    var current = matrix_identity_float4x4
    current.columns.3 = SIMD4<Float>(3.25, -1.5, 0.75, 1)

    let pose = ARKitPoseFrame.pose(reference: reference, current: current)

    XCTAssertEqual(pose.position.x, 0.25, accuracy: 0.0001)
    XCTAssertEqual(pose.position.y, 0.5, accuracy: 0.0001)
    XCTAssertEqual(pose.position.z, -0.25, accuracy: 0.0001)
    XCTAssertEqual(pose.orientation, .identity)
  }

  func testARKitConfigurationDisablesUnusedCaptureFeatures() {
    let configuration = ARKitPoseProvider.makeConfiguration()

    XCTAssertFalse(configuration.isLightEstimationEnabled)
    XCTAssertFalse(configuration.providesAudioData)
    XCTAssertTrue(configuration.frameSemantics.isEmpty)
    XCTAssertFalse(configuration.videoHDRAllowed)
    XCTAssertEqual(configuration.environmentTexturing, .none)
    XCTAssertFalse(configuration.wantsHDREnvironmentTextures)
  }

  func testSimulatorEmitsNewMonotonicSamplesAndStops() {
    let provider = SimulatedPoseProvider(sampleRateHz: 120)
    let collector = SampleCollector()
    let received = expectation(description: "three samples")
    received.expectedFulfillmentCount = 3
    provider.start { sample in
      collector.append(sample)
      received.fulfill()
    }

    wait(for: [received], timeout: 1)
    provider.stop()
    let samplesAtStop = collector.samples
    Thread.sleep(forTimeInterval: 0.05)

    XCTAssertEqual(provider.kind, .simulated)
    XCTAssertGreaterThanOrEqual(samplesAtStop.count, 3)
    XCTAssertEqual(collector.samples.count, samplesAtStop.count)
    XCTAssertTrue(
      zip(samplesAtStop, samplesAtStop.dropFirst()).allSatisfy {
        $0.timestampNs < $1.timestampNs
      }
    )
    XCTAssertTrue(samplesAtStop.allSatisfy { $0.trackingQuality == .normal })
  }

  func testStopIsIdempotent() {
    let provider = SimulatedPoseProvider()

    provider.stop()
    provider.stop()
  }
}
