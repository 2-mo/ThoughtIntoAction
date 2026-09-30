import XCTest

@testable import RobotTeleop

final class ClockSyncTests: XCTestCase {
  func testFourTimestampSampleMatchesProtocolFormula() throws {
    let sample = try ClockSample.calculate(
      clientSendTimeNs: 100,
      serverReceiveTimeNs: 130,
      serverSendTimeNs: 150,
      clientReceiveTimeNs: 200
    )

    XCTAssertEqual(sample.rttNs, 80)
    XCTAssertEqual(sample.serverMinusClientNs, -10)
  }

  func testEstimatorUsesMedianOfLowestRTTSamples() throws {
    let estimator = ClockSyncEstimator(
      maximumRTTNs: 100,
      capacity: 4,
      lowRTTSampleCount: 2
    )

    XCTAssertTrue(estimator.add(exchange(offsetNs: 20, oneWayDelayNs: 5)))
    XCTAssertTrue(estimator.add(exchange(offsetNs: 40, oneWayDelayNs: 10)))
    XCTAssertTrue(estimator.add(exchange(offsetNs: 500, oneWayDelayNs: 40)))

    XCTAssertEqual(estimator.sampleCount, 3)
    XCTAssertEqual(estimator.minimumRTTNs, 10)
    XCTAssertEqual(estimator.offsetNs, 30)
  }

  func testEstimatorRejectsImpossibleAndExcessiveSamples() throws {
    let estimator = ClockSyncEstimator(
      maximumRTTNs: 20,
      capacity: 4,
      lowRTTSampleCount: 2
    )

    XCTAssertFalse(
      estimator.add(
        clientSendTimeNs: 100,
        serverReceiveTimeNs: 130,
        serverSendTimeNs: 200,
        clientReceiveTimeNs: 150
      )
    )
    XCTAssertFalse(estimator.add(exchange(offsetNs: 10, oneWayDelayNs: 20)))
    XCTAssertEqual(estimator.rejectedSampleCount, 2)
    XCTAssertNil(estimator.offsetNs)
  }

  private func exchange(offsetNs: UInt64, oneWayDelayNs: UInt64) -> ClockSample {
    try! ClockSample.calculate(
      clientSendTimeNs: 1_000,
      serverReceiveTimeNs: 1_000 + offsetNs + oneWayDelayNs,
      serverSendTimeNs: 1_000 + offsetNs + oneWayDelayNs + 2,
      clientReceiveTimeNs: 1_000 + 2 * oneWayDelayNs + 2
    )
  }
}
