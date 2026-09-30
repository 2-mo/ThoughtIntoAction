import Foundation

final class SimulatedPoseProvider: PoseProvider, @unchecked Sendable {
  let kind = PoseProviderKind.simulated

  private let sampleRateHz: Double
  private let queue = DispatchQueue(
    label: "com.mo.robotteleop.tracking.simulated",
    qos: .userInteractive
  )
  private let lock = NSLock()
  private var timer: DispatchSourceTimer?
  private var handler: (@Sendable (PoseSample) -> Void)?
  private var startedAtNs: UInt64 = 0

  init(sampleRateHz: Double = 60) {
    precondition(sampleRateHz.isFinite && sampleRateHz > 0)
    self.sampleRateHz = sampleRateHz
  }

  static func resolveSampleRateHz(environment: [String: String]) -> Double {
    guard let rawRate = environment["ROBOT_TELEOP_SIMULATED_RATE_HZ"],
      let rate = Double(rawRate),
      rate.isFinite,
      rate > 0
    else {
      return 60
    }
    return rate
  }

  func start(handler: @escaping @Sendable (PoseSample) -> Void) {
    lock.lock()
    guard timer == nil else {
      lock.unlock()
      return
    }
    self.handler = handler
    startedAtNs = DispatchTime.now().uptimeNanoseconds
    let timer = DispatchSource.makeTimerSource(queue: queue)
    self.timer = timer
    lock.unlock()

    let intervalNs = max(1, Int(1_000_000_000 / sampleRateHz))
    timer.schedule(
      deadline: .now(),
      repeating: .nanoseconds(intervalNs),
      leeway: .microseconds(250)
    )
    timer.setEventHandler { [weak self] in
      self?.emitSample()
    }
    timer.activate()
  }

  func stop() {
    lock.lock()
    let timer = timer
    self.timer = nil
    handler = nil
    lock.unlock()
    timer?.cancel()
  }

  private func emitSample() {
    lock.lock()
    guard timer != nil, let handler else {
      lock.unlock()
      return
    }
    let timestampNs = DispatchTime.now().uptimeNanoseconds
    let elapsed = Double(timestampNs - startedAtNs) / 1_000_000_000
    lock.unlock()

    let phase = Float(elapsed * 0.8)
    let yaw = 0.08 * sin(phase * 0.7)
    handler(
      PoseSample(
        timestampNs: timestampNs,
        pose: TeleopPose(
          position: TeleopVector3(
            x: 0.015 * sin(phase),
            y: 0.010 * sin(phase * 0.5),
            z: 0.012 * cos(phase * 0.8)
          ),
          orientation: TeleopQuaternion(
            x: 0,
            y: 0,
            z: sin(yaw / 2),
            w: cos(yaw / 2)
          )
        ),
        linearVelocity: nil,
        angularVelocity: nil,
        trackingQuality: .normal
      )
    )
  }
}
