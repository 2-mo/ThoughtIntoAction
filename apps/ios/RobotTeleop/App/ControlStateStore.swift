import Foundation

enum ControlDefaults {
  static let positionScale: Float = 1
  static let rotationScale: Float = 1
  static let maximumPositionScale: Float = 2
  static let maximumRotationScale: Float = 1
}

struct ControlSnapshot: Equatable, Sendable {
  let clutchActive: Bool
  let recenterRequested: Bool
  let homeRequested: Bool
  let jointZeroRequested: Bool
  let estopRequested: Bool
  let estopResetPending: Bool
  let gripper: Float
  let positionScale: Float
  let rotationScale: Float
}

final class ControlStateStore: @unchecked Sendable {
  private struct State {
    var clutchActive = false
    var recenterRequested = false
    var homeRequested = false
    var jointZeroRequested = false
    var estopRequested = false
    var estopResetPending = false
    var gripper: Float = 0
    var positionScale: Float = ControlDefaults.positionScale
    var rotationScale: Float = ControlDefaults.rotationScale
  }

  private let lock = NSLock()
  private var state = State()

  @discardableResult
  func setClutchActive(_ active: Bool) -> Bool {
    lock.withLock {
      let newValue = active && !state.estopRequested && !state.estopResetPending
      let changed = state.clutchActive != newValue
      state.clutchActive = newValue
      return changed
    }
  }

  func releaseClutch() {
    lock.withLock {
      state.clutchActive = false
    }
  }

  func deactivate() {
    lock.withLock {
      state.clutchActive = false
      state.recenterRequested = false
      state.homeRequested = false
      state.jointZeroRequested = false
      if state.estopResetPending {
        state.estopRequested = true
        state.estopResetPending = false
      }
    }
  }

  func requestRecenter() {
    lock.withLock {
      guard !state.estopRequested && !state.estopResetPending else { return }
      state.recenterRequested = true
    }
  }

  func requestHome() {
    lock.withLock {
      guard !state.estopRequested && !state.estopResetPending else { return }
      // Home is one-shot; the next clutch edge explicitly re-arms control.
      state.clutchActive = false
      state.homeRequested = true
      state.jointZeroRequested = false
    }
  }

  func requestJointZero() {
    lock.withLock {
      guard !state.estopRequested && !state.estopResetPending else { return }
      state.clutchActive = false
      state.homeRequested = false
      state.jointZeroRequested = true
    }
  }

  @discardableResult
  func latchEstop() -> Bool {
    lock.withLock {
      let changed = !state.estopRequested
      state.estopRequested = true
      state.estopResetPending = false
      state.clutchActive = false
      state.recenterRequested = false
      state.homeRequested = false
      state.jointZeroRequested = false
      return changed
    }
  }

  @discardableResult
  func beginEstopReset() -> Bool {
    lock.withLock {
      guard !state.estopResetPending else { return false }
      state.estopRequested = false
      state.estopResetPending = true
      state.clutchActive = false
      state.recenterRequested = false
      state.homeRequested = false
      state.jointZeroRequested = false
      return true
    }
  }

  @discardableResult
  func completeEstopReset() -> Bool {
    lock.withLock {
      guard state.estopResetPending else { return false }
      state.estopResetPending = false
      state.clutchActive = false
      state.recenterRequested = false
      state.homeRequested = false
      state.jointZeroRequested = false
      return true
    }
  }

  func cancelPendingEstopReset() {
    lock.withLock {
      guard state.estopResetPending else { return }
      state.estopRequested = true
      state.estopResetPending = false
      state.clutchActive = false
      state.recenterRequested = false
      state.homeRequested = false
      state.jointZeroRequested = false
    }
  }

  func setGripper(_ value: Float) {
    guard value.isFinite else { return }
    lock.withLock {
      guard !state.estopRequested else { return }
      state.gripper = min(max(value, 0), 1)
    }
  }

  func setMappingScales(position: Float, rotation: Float) {
    guard position.isFinite, rotation.isFinite else { return }
    lock.withLock {
      guard !state.estopRequested else { return }
      state.positionScale = max(position, 0)
      state.rotationScale = min(max(rotation, 0), ControlDefaults.maximumRotationScale)
    }
  }

  var snapshot: ControlSnapshot {
    lock.withLock { makeSnapshot() }
  }

  func snapshotForPose() -> ControlSnapshot {
    lock.withLock {
      let snapshot = makeSnapshot()
      state.recenterRequested = false
      state.homeRequested = false
      state.jointZeroRequested = false
      return snapshot
    }
  }

  private func makeSnapshot() -> ControlSnapshot {
    ControlSnapshot(
      clutchActive: state.clutchActive,
      recenterRequested: state.recenterRequested,
      homeRequested: state.homeRequested,
      jointZeroRequested: state.jointZeroRequested,
      estopRequested: state.estopRequested,
      estopResetPending: state.estopResetPending,
      gripper: state.gripper,
      positionScale: state.positionScale,
      rotationScale: state.rotationScale
    )
  }
}
