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
    assertValue("就绪", for: "status.network", in: app, timeout: 5)
    assertValue("正常", for: "status.tracking", in: app, timeout: 5)
    assertValueIsNotPlaceholder(for: "status.sequence", in: app, timeout: 5)
    guard valueIsNotPlaceholder(for: "status.rtt", in: app, timeout: 3) else {
      app.terminate()
      throw XCTSkip("Start the Python pose monitor on UDP port \(port) for live validation")
    }
    assertValue("已同步", for: "mapping.settings", in: app, timeout: 5)

    keepScreenshot(named: "iPhone-16-Pro-Max-Connected")
    app.buttons["connection.settings"].tap()
    let disconnect = app.buttons["connection.toggle"]
    XCTAssertTrue(disconnect.waitForExistence(timeout: 2))
    XCTAssertEqual(disconnect.label, "断开连接")
    disconnect.tap()
    app.buttons["完成"].tap()
    assertValue("未连接", for: "status.network", in: app, timeout: 5)
    app.terminate()
  }

  @MainActor
  private func assertPrimaryControls(in app: XCUIApplication) {
    XCTAssertTrue(app.staticTexts["知行有灵"].waitForExistence(timeout: 3))
    XCTAssertTrue(app.buttons["connection.settings"].isHittable)
    for identifier in ["status.tracking", "status.send-fps", "status.sequence",
                       "status.rtt", "status.session", "status.network"] {
      let card = app.descendants(matching: .any)[identifier]
      XCTAssertTrue(card.exists, identifier)
      XCTAssertTrue(app.frame.contains(card.frame), identifier)
    }
    for label in ["开始遥操", "停止遥操", "回零", "预抓取", "方向对齐"] {
      XCTAssertTrue(app.buttons[label].exists, label)
      XCTAssertTrue(app.frame.contains(app.buttons[label].frame), label)
    }
    let clutch = app.descendants(matching: .any)["control.clutch"]
    XCTAssertTrue(clutch.exists)
    XCTAssertTrue(app.frame.contains(clutch.frame))
    XCTAssertGreaterThanOrEqual(clutch.frame.height, 148)
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
