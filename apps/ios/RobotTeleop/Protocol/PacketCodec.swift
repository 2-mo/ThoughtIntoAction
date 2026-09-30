import Foundation

enum PacketCodec {
  static func encode(_ packet: TeleopPacket) throws -> Data {
    var writer = LittleEndianWriter()

    switch packet {
    case .pose(let posePacket):
      try encodePose(posePacket, into: &writer)
    case .ping(let pingPacket):
      try validateIdentity(source: pingPacket.sourceType, sessionID: pingPacket.sessionID)
      encodeHeader(
        type: .ping,
        source: pingPacket.sourceType,
        flags: [],
        sequence: pingPacket.sequence,
        sessionID: pingPacket.sessionID,
        clientTimeNs: pingPacket.clientTimeNs,
        into: &writer
      )
    case .pong(let pongPacket):
      try validateIdentity(source: pongPacket.sourceType, sessionID: pongPacket.sessionID)
      guard pongPacket.serverSendTimeNs >= pongPacket.serverReceiveTimeNs else {
        throw PacketCodecError.fieldRange
      }
      encodeHeader(
        type: .pong,
        source: pongPacket.sourceType,
        flags: [],
        sequence: pongPacket.sequence,
        sessionID: pongPacket.sessionID,
        clientTimeNs: pongPacket.clientTimeNs,
        into: &writer
      )
      writer.append(pongPacket.serverReceiveTimeNs)
      writer.append(pongPacket.serverSendTimeNs)
    case .settings(let settingsPacket):
      try encodeSettings(
        type: .settings,
        source: settingsPacket.sourceType,
        sequence: settingsPacket.sequence,
        sessionID: settingsPacket.sessionID,
        clientTimeNs: settingsPacket.clientTimeNs,
        positionScale: settingsPacket.positionScale,
        rotationScale: settingsPacket.rotationScale,
        into: &writer
      )
    case .settingsAck(let settingsPacket):
      try encodeSettings(
        type: .settingsAck,
        source: settingsPacket.sourceType,
        sequence: settingsPacket.sequence,
        sessionID: settingsPacket.sessionID,
        clientTimeNs: settingsPacket.clientTimeNs,
        positionScale: settingsPacket.positionScale,
        rotationScale: settingsPacket.rotationScale,
        into: &writer
      )
    case .estopResetRequest(let resetPacket):
      try encodeEstopReset(
        type: .estopResetRequest,
        source: resetPacket.sourceType,
        sequence: resetPacket.sequence,
        sessionID: resetPacket.sessionID,
        clientTimeNs: resetPacket.clientTimeNs,
        into: &writer
      )
    case .estopResetAck(let resetPacket):
      try encodeEstopReset(
        type: .estopResetAck,
        source: resetPacket.sourceType,
        sequence: resetPacket.sequence,
        sessionID: resetPacket.sessionID,
        clientTimeNs: resetPacket.clientTimeNs,
        into: &writer
      )
    }

    return writer.data
  }

  static func decode(_ data: Data) throws -> TeleopPacket {
    guard data.count >= ProtocolConstants.headerSize else {
      throw PacketCodecError.length
    }

    var reader = LittleEndianReader(data: data)
    let magic: UInt32 = try reader.read()
    guard magic == ProtocolConstants.magic else {
      throw PacketCodecError.magic
    }

    let major: UInt16 = try reader.read()
    let minor: UInt16 = try reader.read()
    guard
      major == ProtocolConstants.majorVersion,
      minor >= ProtocolConstants.minimumSupportedMinorVersion,
      minor <= ProtocolConstants.minorVersion
    else {
      throw PacketCodecError.version
    }

    let declaredSize = Int(try reader.read() as UInt16)
    let rawType: UInt16 = try reader.read()
    guard let packetType = PacketType(rawValue: rawType) else {
      throw PacketCodecError.type
    }
    guard declaredSize == packetType.packetSize, data.count == packetType.packetSize else {
      throw PacketCodecError.size
    }

    let rawSource: UInt16 = try reader.read()
    guard let source = SourceType(rawValue: rawSource) else {
      throw PacketCodecError.source
    }
    if [.pose, .settings, .settingsAck, .estopResetRequest, .estopResetAck].contains(packetType),
      source == .unknown
    {
      throw PacketCodecError.source
    }

    let reserved: UInt16 = try reader.read()
    guard reserved == 0 else {
      throw PacketCodecError.reserved
    }

    let rawFlags: UInt32 = try reader.read()
    guard rawFlags & ~PacketFlags.knownMask == 0 else {
      throw PacketCodecError.flags
    }
    guard packetType == .pose || rawFlags == 0 else {
      throw PacketCodecError.flags
    }

    let sequence: UInt32 = try reader.read()
    let sessionID: UInt32 = try reader.read()
    guard sessionID != 0 else {
      throw PacketCodecError.fieldRange
    }
    let clientTimeNs: UInt64 = try reader.read()

    switch packetType {
    case .pose:
      return try decodePose(
        source: source,
        flags: PacketFlags(rawValue: rawFlags),
        sequence: sequence,
        sessionID: sessionID,
        clientTimeNs: clientTimeNs,
        reader: &reader
      )
    case .ping:
      return .ping(
        PingPacket(
          sourceType: source,
          sequence: sequence,
          sessionID: sessionID,
          clientTimeNs: clientTimeNs
        )
      )
    case .pong:
      let serverReceiveTimeNs: UInt64 = try reader.read()
      let serverSendTimeNs: UInt64 = try reader.read()
      guard serverSendTimeNs >= serverReceiveTimeNs else {
        throw PacketCodecError.fieldRange
      }
      return .pong(
        PongPacket(
          sourceType: source,
          sequence: sequence,
          sessionID: sessionID,
          clientTimeNs: clientTimeNs,
          serverReceiveTimeNs: serverReceiveTimeNs,
          serverSendTimeNs: serverSendTimeNs
        )
      )
    case .settings, .settingsAck:
      let positionScale = try reader.readFloat()
      let rotationScale = try reader.readFloat()
      try validateScales(position: positionScale, rotation: rotationScale)
      if packetType == .settings {
        return .settings(
          SettingsPacket(
            sourceType: source,
            sequence: sequence,
            sessionID: sessionID,
            clientTimeNs: clientTimeNs,
            positionScale: positionScale,
            rotationScale: rotationScale
          )
        )
      }
      return .settingsAck(
        SettingsAckPacket(
          sourceType: source,
          sequence: sequence,
          sessionID: sessionID,
          clientTimeNs: clientTimeNs,
          positionScale: positionScale,
          rotationScale: rotationScale
        )
      )
    case .estopResetRequest:
      return .estopResetRequest(
        EStopResetRequestPacket(
          sourceType: source,
          sequence: sequence,
          sessionID: sessionID,
          clientTimeNs: clientTimeNs
        )
      )
    case .estopResetAck:
      return .estopResetAck(
        EStopResetAckPacket(
          sourceType: source,
          sequence: sequence,
          sessionID: sessionID,
          clientTimeNs: clientTimeNs
        )
      )
    }
  }

  private static func encodePose(_ packet: PosePacket, into writer: inout LittleEndianWriter)
    throws
  {
    try validateIdentity(
      source: packet.sourceType, sessionID: packet.sessionID, requiresSource: true)
    guard !packet.clutchActive || (packet.connected && packet.trackingQuality == .normal)
    else {
      throw PacketCodecError.flags
    }

    let quaternion = try normalized(packet.pose.orientation)
    let linearVelocity = packet.linearVelocity ?? .zero
    let angularVelocity = packet.angularVelocity ?? .zero
    let gripper = packet.gripper ?? 0
    let values = [
      packet.pose.position.x,
      packet.pose.position.y,
      packet.pose.position.z,
      quaternion.x,
      quaternion.y,
      quaternion.z,
      quaternion.w,
      linearVelocity.x,
      linearVelocity.y,
      linearVelocity.z,
      angularVelocity.x,
      angularVelocity.y,
      angularVelocity.z,
      gripper,
    ]
    guard values.allSatisfy(\.isFinite) else {
      throw PacketCodecError.nonFinite
    }
    if packet.gripper != nil, !(0...1).contains(gripper) {
      throw PacketCodecError.gripper
    }

    var flags: PacketFlags = []
    if packet.connected { flags.insert(.connected) }
    if packet.clutchActive { flags.insert(.clutchActive) }
    if packet.recenterRequested { flags.insert(.recenter) }
    if packet.homeRequested { flags.insert(.home) }
    if packet.jointZeroRequested { flags.insert(.jointZero) }
    if packet.estopRequested { flags.insert(.estop) }
    if packet.trackingQuality != .unavailable { flags.insert(.trackingValid) }
    if packet.gripper != nil { flags.insert(.gripperValid) }
    if packet.linearVelocity != nil { flags.insert(.linearVelocityValid) }
    if packet.angularVelocity != nil { flags.insert(.angularVelocityValid) }

    encodeHeader(
      type: .pose,
      source: packet.sourceType,
      flags: flags,
      sequence: packet.sequence,
      sessionID: packet.sessionID,
      clientTimeNs: packet.clientTimeNs,
      into: &writer
    )
    for value in values {
      writer.append(value)
    }
    writer.append(packet.trackingQuality.rawValue)
    writer.append(UInt8(0))
    writer.append(UInt8(0))
    writer.append(UInt8(0))
  }

  private static func decodePose(
    source: SourceType,
    flags: PacketFlags,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64,
    reader: inout LittleEndianReader
  ) throws -> TeleopPacket {
    var values: [Float] = []
    values.reserveCapacity(14)
    for _ in 0..<14 {
      values.append(try reader.readFloat())
    }
    guard values.allSatisfy(\.isFinite) else {
      throw PacketCodecError.nonFinite
    }

    let rawTracking: UInt8 = try reader.read()
    guard let trackingQuality = TrackingQuality(rawValue: rawTracking) else {
      throw PacketCodecError.tracking
    }
    let reserved0: UInt8 = try reader.read()
    let reserved1: UInt8 = try reader.read()
    let reserved2: UInt8 = try reader.read()
    guard reserved0 == 0, reserved1 == 0, reserved2 == 0 else {
      throw PacketCodecError.reserved
    }

    let quaternion = try normalized(
      TeleopQuaternion(x: values[3], y: values[4], z: values[5], w: values[6])
    )
    let trackingValid = flags.contains(.trackingValid)
    guard trackingValid == (trackingQuality != .unavailable) else {
      throw PacketCodecError.tracking
    }
    let connected = flags.contains(.connected)
    let clutchActive = flags.contains(.clutchActive)
    guard !clutchActive || (connected && trackingQuality == .normal) else {
      throw PacketCodecError.flags
    }

    let rawGripper = values[13]
    let gripper: Float? = flags.contains(.gripperValid) ? rawGripper : nil
    if let gripper, !(0...1).contains(gripper) {
      throw PacketCodecError.gripper
    }

    return .pose(
      PosePacket(
        sourceType: source,
        sequence: sequence,
        sessionID: sessionID,
        clientTimeNs: clientTimeNs,
        pose: TeleopPose(
          position: TeleopVector3(x: values[0], y: values[1], z: values[2]),
          orientation: quaternion
        ),
        linearVelocity: flags.contains(.linearVelocityValid)
          ? TeleopVector3(x: values[7], y: values[8], z: values[9]) : nil,
        angularVelocity: flags.contains(.angularVelocityValid)
          ? TeleopVector3(x: values[10], y: values[11], z: values[12]) : nil,
        connected: connected,
        clutchActive: clutchActive,
        recenterRequested: flags.contains(.recenter),
        estopRequested: flags.contains(.estop),
        gripper: gripper,
        trackingQuality: trackingQuality,
        homeRequested: flags.contains(.home),
        jointZeroRequested: flags.contains(.jointZero)
      )
    )
  }

  private static func encodeSettings(
    type: PacketType,
    source: SourceType,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64,
    positionScale: Float,
    rotationScale: Float,
    into writer: inout LittleEndianWriter
  ) throws {
    try validateIdentity(source: source, sessionID: sessionID, requiresSource: true)
    try validateScales(position: positionScale, rotation: rotationScale)
    encodeHeader(
      type: type,
      source: source,
      flags: [],
      sequence: sequence,
      sessionID: sessionID,
      clientTimeNs: clientTimeNs,
      into: &writer
    )
    writer.append(positionScale)
    writer.append(rotationScale)
  }

  private static func encodeHeader(
    type: PacketType,
    source: SourceType,
    flags: PacketFlags,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64,
    into writer: inout LittleEndianWriter
  ) {
    writer.append(ProtocolConstants.magic)
    writer.append(ProtocolConstants.majorVersion)
    writer.append(ProtocolConstants.minorVersion)
    writer.append(UInt16(type.packetSize))
    writer.append(type.rawValue)
    writer.append(source.rawValue)
    writer.append(UInt16(0))
    writer.append(flags.rawValue)
    writer.append(sequence)
    writer.append(sessionID)
    writer.append(clientTimeNs)
  }

  private static func validateIdentity(
    source: SourceType,
    sessionID: UInt32,
    requiresSource: Bool = false
  ) throws {
    if requiresSource, source == .unknown {
      throw PacketCodecError.source
    }
    guard sessionID != 0 else {
      throw PacketCodecError.fieldRange
    }
  }

  private static func validateScales(position: Float, rotation: Float) throws {
    guard position.isFinite, rotation.isFinite else {
      throw PacketCodecError.nonFinite
    }
    guard position >= 0, (0...1).contains(rotation) else {
      throw PacketCodecError.fieldRange
    }
  }

  private static func normalized(_ quaternion: TeleopQuaternion) throws -> TeleopQuaternion {
    let values = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    guard values.allSatisfy(\.isFinite) else {
      throw PacketCodecError.nonFinite
    }
    let norm = sqrt(values.reduce(Float(0)) { $0 + $1 * $1 })
    guard ProtocolConstants.quaternionNormRange.contains(norm) else {
      throw PacketCodecError.quaternion
    }
    return TeleopQuaternion(
      x: quaternion.x / norm,
      y: quaternion.y / norm,
      z: quaternion.z / norm,
      w: quaternion.w / norm
    )
  }

  private static func encodeEstopReset(
    type: PacketType,
    source: SourceType,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64,
    into writer: inout LittleEndianWriter
  ) throws {
    try validateIdentity(source: source, sessionID: sessionID, requiresSource: true)
    encodeHeader(
      type: type,
      source: source,
      flags: [],
      sequence: sequence,
      sessionID: sessionID,
      clientTimeNs: clientTimeNs,
      into: &writer
    )
  }
}

private struct LittleEndianWriter {
  private(set) var data = Data()

  mutating func append<T: FixedWidthInteger>(_ value: T) {
    var littleEndian = value.littleEndian
    withUnsafeBytes(of: &littleEndian) { bytes in
      data.append(contentsOf: bytes)
    }
  }

  mutating func append(_ value: Float) {
    append(value.bitPattern)
  }
}

private struct LittleEndianReader {
  let data: Data
  private(set) var offset = 0

  mutating func read<T: FixedWidthInteger>() throws -> T {
    let size = MemoryLayout<T>.size
    guard offset + size <= data.count else {
      throw PacketCodecError.length
    }
    let raw = data.withUnsafeBytes { bytes in
      bytes.loadUnaligned(fromByteOffset: offset, as: T.self)
    }
    offset += size
    return T(littleEndian: raw)
  }

  mutating func readFloat() throws -> Float {
    let bits: UInt32 = try read()
    return Float(bitPattern: bits)
  }
}
