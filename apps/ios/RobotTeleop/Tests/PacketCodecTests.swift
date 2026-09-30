import Foundation
import XCTest

@testable import RobotTeleop

final class PacketCodecTests: XCTestCase {
  private let goldenPose = PosePacket(
    sequence: 0x1020_3040,
    sessionID: 0x5060_7080,
    clientTimeNs: 0x0102_0304_0506_0708,
    pose: TeleopPose(
      position: TeleopVector3(x: 0.25, y: -0.5, z: 1),
      orientation: TeleopQuaternion(x: 0.5, y: 0.5, z: 0.5, w: -0.5)
    ),
    linearVelocity: TeleopVector3(x: 0.125, y: -0.25, z: 0.5),
    angularVelocity: TeleopVector3(x: -0.125, y: 0.25, z: -0.5),
    connected: true,
    clutchActive: true,
    recenterRequested: true,
    estopRequested: false,
    gripper: 0.75,
    trackingQuality: .normal
  )

  func testAllSwiftEncodingsMatchSharedGoldenBytes() throws {
    let packets: [(String, TeleopPacket)] = [
      ("pose_v1.bin", .pose(goldenPose)),
      (
        "ping_v1.bin",
        .ping(
          PingPacket(
            sequence: 0xaabb_ccdd,
            sessionID: 0x5060_7080,
            clientTimeNs: 0x1112_1314_1516_1718
          )
        )
      ),
      (
        "pong_v1.bin",
        .pong(
          PongPacket(
            sequence: 0xaabb_ccdd,
            sessionID: 0x5060_7080,
            clientTimeNs: 0x1112_1314_1516_1718,
            serverReceiveTimeNs: 0x2122_2324_2526_2728,
            serverSendTimeNs: 0x3132_3334_3536_3738
          )
        )
      ),
      (
        "settings_v1.bin",
        .settings(
          SettingsPacket(
            sequence: 0x0a0b_0c0d,
            sessionID: 0x5060_7080,
            clientTimeNs: 0x4142_4344_4546_4748,
            positionScale: 0.75,
            rotationScale: 0.5
          )
        )
      ),
      (
        "settings_ack_v1.bin",
        .settingsAck(
          SettingsAckPacket(
            sequence: 0x0a0b_0c0d,
            sessionID: 0x5060_7080,
            clientTimeNs: 0x4142_4344_4546_4748,
            positionScale: 0.75,
            rotationScale: 0.5
          )
        )
      ),
      (
        "estop_reset_request_v1.bin",
        .estopResetRequest(
          EStopResetRequestPacket(
            sequence: 0xd0d1_d2d3,
            sessionID: 0x5060_7080,
            clientTimeNs: 0x5152_5354_5556_5758
          )
        )
      ),
      (
        "estop_reset_ack_v1.bin",
        .estopResetAck(
          EStopResetAckPacket(
            sequence: 0xd0d1_d2d3,
            sessionID: 0x5060_7080,
            clientTimeNs: 0x5152_5354_5556_5758
          )
        )
      ),
    ]

    for (filename, packet) in packets {
      let golden = try fixture(filename)
      XCTAssertEqual(try PacketCodec.encode(packet), golden, filename)
      XCTAssertEqual(try PacketCodec.decode(golden), packet, filename)
    }
  }

  func testGoldenPoseFreezesQuaternionAsXYZW() throws {
    let data = try fixture("pose_v1.bin")

    XCTAssertEqual(float(at: 48, in: data), 0.5)
    XCTAssertEqual(float(at: 52, in: data), 0.5)
    XCTAssertEqual(float(at: 56, in: data), 0.5)
    XCTAssertEqual(float(at: 60, in: data), -0.5)
  }

  func testMalformedPacketsAreRejectedWithStableReasons() throws {
    let pose = try fixture("pose_v1.bin")
    var badMagic = pose
    badMagic.replaceSubrange(0..<4, with: [0, 0, 0, 0])
    var badVersion = pose
    badVersion.replaceSubrange(4..<6, with: [2, 0])
    var nonFinite = pose
    writeFloat(.nan, at: 36, in: &nonFinite)
    var badQuaternion = pose
    for offset in stride(from: 48, through: 60, by: 4) {
      writeFloat(0, at: offset, in: &badQuaternion)
    }

    assertRejected(Data(), as: .length)
    assertRejected(pose.dropLast(), as: .size)
    assertRejected(badMagic, as: .magic)
    assertRejected(badVersion, as: .version)
    assertRejected(nonFinite, as: .nonFinite)
    assertRejected(badQuaternion, as: .quaternion)
  }

  func testManifestDeclaresSevenVersionOneMessages() throws {
    let object = try JSONSerialization.jsonObject(with: fixture("manifest.json"))
    let manifest = try XCTUnwrap(object as? [String: Any])
    let protocolFields = try XCTUnwrap(manifest["protocol"] as? [String: Any])
    let messages = try XCTUnwrap(manifest["messages"] as? [String: Any])

    XCTAssertEqual(protocolFields["major"] as? Int, 1)
    XCTAssertEqual(protocolFields["minor"] as? Int, 1)
    XCTAssertEqual(protocolFields["quaternion_order"] as? String, "xyzw")
    XCTAssertEqual(messages.count, 7)
  }

  func testVersionOnePointZeroMessagesRemainDecodable() throws {
    var legacy = try fixture("pose_v1.bin")
    writeUInt16(0, at: 6, in: &legacy)

    XCTAssertEqual(try PacketCodec.decode(legacy), .pose(goldenPose))
  }

  private func fixture(_ filename: String) throws -> Data {
    let parts = filename.split(separator: ".", maxSplits: 1).map(String.init)
    let name = parts[0]
    let extensionName = parts.count == 2 ? parts[1] : nil
    let url = try XCTUnwrap(
      Bundle(for: Self.self).url(forResource: name, withExtension: extensionName),
      "Missing test resource: \(filename)"
    )
    return try Data(contentsOf: url)
  }

  private func float(at offset: Int, in data: Data) -> Float {
    let bits = data.withUnsafeBytes {
      $0.loadUnaligned(fromByteOffset: offset, as: UInt32.self)
    }
    return Float(bitPattern: UInt32(littleEndian: bits))
  }

  private func writeFloat(_ value: Float, at offset: Int, in data: inout Data) {
    var bits = value.bitPattern.littleEndian
    withUnsafeBytes(of: &bits) { bytes in
      data.replaceSubrange(offset..<(offset + 4), with: bytes)
    }
  }

  private func writeUInt16(_ value: UInt16, at offset: Int, in data: inout Data) {
    var littleEndian = value.littleEndian
    withUnsafeBytes(of: &littleEndian) { bytes in
      data.replaceSubrange(offset..<(offset + 2), with: bytes)
    }
  }

  private func assertRejected(
    _ data: some DataProtocol,
    as expected: PacketCodecError
  ) {
    XCTAssertThrowsError(try PacketCodec.decode(Data(data))) { error in
      XCTAssertEqual(error as? PacketCodecError, expected)
    }
  }
}
