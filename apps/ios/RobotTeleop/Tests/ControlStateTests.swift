import XCTest

@testable import RobotTeleop

final class ControlStateTests: XCTestCase {
  func testRecenterIsConsumedByExactlyOnePoseSnapshot() {
    let store = ControlStateStore()

    store.requestRecenter()

    XCTAssertTrue(store.snapshotForPose().recenterRequested)
    XCTAssertFalse(store.snapshotForPose().recenterRequested)
  }

  func testHomeReleasesClutchAndIsConsumedByExactlyOnePoseSnapshot() {
    let store = ControlStateStore()
    store.setClutchActive(true)

    store.requestHome()

    let home = store.snapshotForPose()
    XCTAssertTrue(home.homeRequested)
    XCTAssertFalse(home.clutchActive)
    XCTAssertFalse(store.snapshotForPose().homeRequested)
  }

  func testJointZeroIsOneShotAndReleasesClutch() {
    let store = ControlStateStore()
    store.setClutchActive(true)

    store.requestJointZero()

    let zero = store.snapshotForPose()
    XCTAssertTrue(zero.jointZeroRequested)
    XCTAssertFalse(zero.clutchActive)
    XCTAssertFalse(store.snapshotForPose().jointZeroRequested)
  }

  func testEstopLatchesAndClearsMotionControls() {
    let store = ControlStateStore()
    store.setClutchActive(true)
    store.requestRecenter()

    store.latchEstop()
    store.setClutchActive(true)
    store.requestRecenter()

    let snapshot = store.snapshotForPose()
    XCTAssertTrue(snapshot.estopRequested)
    XCTAssertFalse(snapshot.clutchActive)
    XCTAssertFalse(snapshot.recenterRequested)
  }

  func testClutchCanBeReleasedIdempotently() {
    let store = ControlStateStore()
    store.setClutchActive(true)

    store.releaseClutch()
    store.releaseClutch()

    XCTAssertFalse(store.snapshot.clutchActive)
  }

  func testEstopResetPreparationKeepsControlLockedUntilAcknowledged() {
    let store = ControlStateStore()
    store.latchEstop()

    XCTAssertTrue(store.beginEstopReset())
    XCTAssertFalse(store.snapshot.estopRequested)
    XCTAssertTrue(store.snapshot.estopResetPending)
    XCTAssertFalse(store.snapshot.clutchActive)

    XCTAssertTrue(store.completeEstopReset())
    XCTAssertFalse(store.snapshot.estopRequested)
    XCTAssertFalse(store.snapshot.estopResetPending)
  }

  func testRecoveryCanStartWithoutALocalEstop() {
    let store = ControlStateStore()

    XCTAssertTrue(store.beginEstopReset())
    XCTAssertFalse(store.snapshot.estopRequested)
    XCTAssertTrue(store.snapshot.estopResetPending)
    XCTAssertFalse(store.setClutchActive(true))
  }

  func testDisconnectCancelsPendingResetBackToEstop() {
    let store = ControlStateStore()
    store.latchEstop()
    XCTAssertTrue(store.beginEstopReset())

    store.cancelPendingEstopReset()

    XCTAssertTrue(store.snapshot.estopRequested)
    XCTAssertFalse(store.snapshot.estopResetPending)
  }

  func testControlValuesAreClampedAtTheUIBoundary() {
    let store = ControlStateStore()

    store.setGripper(1.5)
    store.setMappingScales(position: -1, rotation: 2)

    XCTAssertEqual(store.snapshot.gripper, 1)
    XCTAssertEqual(store.snapshot.positionScale, 0)
    XCTAssertEqual(store.snapshot.rotationScale, 1)
  }
}
