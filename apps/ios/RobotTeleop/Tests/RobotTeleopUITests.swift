import XCTest

final class RobotTeleopUITests: XCTestCase {
  override func setUp() {
    continueAfterFailure = false
  }

  @MainActor
  func testPortraitControllerIsFullyVisible() {
    XCUIDevice.shared.orientation = .portrait
    let app = XCUIApplication()
    app.launch()

    assertPrimaryControls(in: app)
    keepScreenshot(named: "iPhone-16-Pro-Max-Portrait")
    app.terminate()
  }

  @MainActor
  func testLandscapeControllerIsFullyVisible() {
    XCUIDevice.shared.orientation = .landscapeLeft
    let app = XCUIApplication()
    app.launch()

    assertPrimaryControls(in: app)
    keepScreenshot(named: "iPhone-16-Pro-Max-Landscape")
    app.terminate()
  }

  @MainActor
  func testSimulatorUDPProtocolRoundTrip() throws {
    try runSimulatorUDPProtocolRoundTrip(simulatedRateHz: nil)
  }

  @MainActor
  func testSimulatorUDPProtocolRoundTripAt120Hz() throws {
    try runSimulatorUDPProtocolRoundTrip(simulatedRateHz: "120")
  }

  @MainActor
  private func runSimulatorUDPProtocolRoundTrip(
    simulatedRateHz: String?
  ) throws {
    let environment = ProcessInfo.processInfo.environment
    let port = environment["ROBOT_TELEOP_NETWORK_TEST_PORT"] ?? "5005"

    XCUIDevice.shared.orientation = .portrait
    let app = XCUIApplication()
    app.launchEnvironment["ROBOT_TELEOP_HOST"] = "127.0.0.1"
    app.launchEnvironment["ROBOT_TELEOP_PORT"] = port
    if let simulatedRateHz {
      app.launchEnvironment["ROBOT_TELEOP_SIMULATED_RATE_HZ"] = simulatedRateHz
    }
    app.launch()

    assertPrimaryControls(in: app)
    if app.buttons["Connect"].exists { app.buttons["Connect"].tap() }

    assertValue("READY", for: "status.network", in: app, timeout: 5)
    assertValue("NORMAL", for: "status.tracking", in: app, timeout: 5)
    assertValueIsNotPlaceholder(for: "status.sequence", in: app, timeout: 5)
    guard valueIsNotPlaceholder(for: "status.rtt", in: app, timeout: 3) else {
      app.terminate()
      throw XCTSkip("Start the Python pose monitor on UDP port \(port) for live validation")
    }
    assertValue("SYNCED", for: "status.settings", in: app, timeout: 5)

    let disconnect = app.buttons["Disconnect"]
    XCTAssertTrue(disconnect.waitForExistence(timeout: 2))
    XCTAssertGreaterThanOrEqual(disconnect.frame.width, 120)
    keepScreenshot(named: "iPhone-16-Pro-Max-Connected")
    disconnect.tap()
    assertValue("OFFLINE", for: "status.network", in: app, timeout: 5)
    app.terminate()
  }

  @MainActor
  private func assertPrimaryControls(in app: XCUIApplication) {
    let titleExists = app.staticTexts["Robot Teleop"].waitForExistence(timeout: 3)
    let connectIsHittable = app.buttons["Connect"].isHittable || app.buttons["Disconnect"].isHittable
    let holdExists = app.descendants(matching: .any)["按住跟随，上滑开夹爪，下滑闭夹爪"].exists
    let startExists = app.buttons["开始遥操"].exists
    let stopExists = app.buttons["停止遥操"].exists
    XCTAssertTrue(titleExists)
    XCTAssertTrue(connectIsHittable)
    XCTAssertTrue(holdExists)
    XCTAssertTrue(startExists)
    XCTAssertTrue(stopExists)
  }

  @MainActor
  private func keepScreenshot(named name: String) {
    let attachment = XCTAttachment(screenshot: XCUIScreen.main.screenshot())
    attachment.name = name
    attachment.lifetime = .keepAlways
    add(attachment)
  }

  @MainActor
  private func assertValue(
    _ value: String,
    for identifier: String,
    in app: XCUIApplication,
    timeout: TimeInterval
  ) {
    let element = app.descendants(matching: .any)[identifier]
    let predicate = NSPredicate(format: "value == %@", value)
    let expectation = XCTNSPredicateExpectation(predicate: predicate, object: element)
    XCTAssertEqual(XCTWaiter.wait(for: [expectation], timeout: timeout), .completed)
  }

  @MainActor
  private func assertValueIsNotPlaceholder(
    for identifier: String,
    in app: XCUIApplication,
    timeout: TimeInterval
  ) {
    XCTAssertTrue(valueIsNotPlaceholder(for: identifier, in: app, timeout: timeout))
  }

  @MainActor
  private func valueIsNotPlaceholder(
    for identifier: String,
    in app: XCUIApplication,
    timeout: TimeInterval
  ) -> Bool {
    let element = app.descendants(matching: .any)[identifier]
    let predicate = NSPredicate(format: "exists == true AND value != nil AND value != '--'")
    let expectation = XCTNSPredicateExpectation(predicate: predicate, object: element)
    return XCTWaiter.wait(for: [expectation], timeout: timeout) == .completed
  }
}
