import Foundation

enum ProtocolConstants {
  static let magic: UInt32 = 0x504c_5452
  static let majorVersion: UInt16 = 1
  static let minimumSupportedMinorVersion: UInt16 = 0
  static let minorVersion: UInt16 = 1

  static let headerSize = 36
  static let posePacketSize = 96
  static let pingPacketSize = 36
  static let pongPacketSize = 52
  static let settingsPacketSize = 44

  static let quaternionNormRange: ClosedRange<Float> = 0.95...1.05
}

enum PacketType: UInt16, Sendable {
  case pose = 1
  case ping = 2
  case pong = 3
  case settings = 4
  case settingsAck = 5
  case estopResetRequest = 6
  case estopResetAck = 7

  var packetSize: Int {
    switch self {
    case .pose:
      ProtocolConstants.posePacketSize
    case .ping, .estopResetRequest, .estopResetAck:
      ProtocolConstants.pingPacketSize
    case .pong:
      ProtocolConstants.pongPacketSize
    case .settings, .settingsAck:
      ProtocolConstants.settingsPacketSize
    }
  }
}

enum SourceType: UInt16, Sendable {
  case unknown = 0
  case arkit = 1
  case vr = 2
  case spaceMouse = 3
  case handTracking = 4
  case teaching = 5
  case replay = 6
  case mock = 7
}

struct PacketFlags: OptionSet, Sendable {
  let rawValue: UInt32

  static let connected = PacketFlags(rawValue: 1 << 0)
  static let clutchActive = PacketFlags(rawValue: 1 << 1)
  static let recenter = PacketFlags(rawValue: 1 << 2)
  static let estop = PacketFlags(rawValue: 1 << 3)
  static let trackingValid = PacketFlags(rawValue: 1 << 4)
  static let gripperValid = PacketFlags(rawValue: 1 << 5)
  static let linearVelocityValid = PacketFlags(rawValue: 1 << 6)
  static let angularVelocityValid = PacketFlags(rawValue: 1 << 7)
  static let home = PacketFlags(rawValue: 1 << 8)
  static let jointZero = PacketFlags(rawValue: 1 << 9)

  static let knownMask: UInt32 = 0x3ff
}

struct PosePacket: Equatable, Sendable {
  let sourceType: SourceType
  let sequence: UInt32
  let sessionID: UInt32
  let clientTimeNs: UInt64
  let pose: TeleopPose
  let linearVelocity: TeleopVector3?
  let angularVelocity: TeleopVector3?
  let connected: Bool
  let clutchActive: Bool
  let recenterRequested: Bool
  let estopRequested: Bool
  let gripper: Float?
  let trackingQuality: TrackingQuality
  let homeRequested: Bool
  let jointZeroRequested: Bool

  init(
    sourceType: SourceType = .arkit,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64,
    pose: TeleopPose,
    linearVelocity: TeleopVector3?,
    angularVelocity: TeleopVector3?,
    connected: Bool,
    clutchActive: Bool,
    recenterRequested: Bool,
    estopRequested: Bool,
    gripper: Float?,
    trackingQuality: TrackingQuality,
    homeRequested: Bool = false,
    jointZeroRequested: Bool = false
  ) {
    self.sourceType = sourceType
    self.sequence = sequence
    self.sessionID = sessionID
    self.clientTimeNs = clientTimeNs
    self.pose = pose
    self.linearVelocity = linearVelocity
    self.angularVelocity = angularVelocity
    self.connected = connected
    self.clutchActive = clutchActive
    self.recenterRequested = recenterRequested
    self.estopRequested = estopRequested
    self.gripper = gripper
    self.trackingQuality = trackingQuality
    self.homeRequested = homeRequested
    self.jointZeroRequested = jointZeroRequested
  }
}

struct PingPacket: Equatable, Sendable {
  let sourceType: SourceType
  let sequence: UInt32
  let sessionID: UInt32
  let clientTimeNs: UInt64

  init(
    sourceType: SourceType = .arkit,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64
  ) {
    self.sourceType = sourceType
    self.sequence = sequence
    self.sessionID = sessionID
    self.clientTimeNs = clientTimeNs
  }
}

struct PongPacket: Equatable, Sendable {
  let sourceType: SourceType
  let sequence: UInt32
  let sessionID: UInt32
  let clientTimeNs: UInt64
  let serverReceiveTimeNs: UInt64
  let serverSendTimeNs: UInt64

  init(
    sourceType: SourceType = .arkit,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64,
    serverReceiveTimeNs: UInt64,
    serverSendTimeNs: UInt64
  ) {
    self.sourceType = sourceType
    self.sequence = sequence
    self.sessionID = sessionID
    self.clientTimeNs = clientTimeNs
    self.serverReceiveTimeNs = serverReceiveTimeNs
    self.serverSendTimeNs = serverSendTimeNs
  }
}

struct SettingsPacket: Equatable, Sendable {
  let sourceType: SourceType
  let sequence: UInt32
  let sessionID: UInt32
  let clientTimeNs: UInt64
  let positionScale: Float
  let rotationScale: Float

  init(
    sourceType: SourceType = .arkit,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64,
    positionScale: Float,
    rotationScale: Float
  ) {
    self.sourceType = sourceType
    self.sequence = sequence
    self.sessionID = sessionID
    self.clientTimeNs = clientTimeNs
    self.positionScale = positionScale
    self.rotationScale = rotationScale
  }
}

struct SettingsAckPacket: Equatable, Sendable {
  let sourceType: SourceType
  let sequence: UInt32
  let sessionID: UInt32
  let clientTimeNs: UInt64
  let positionScale: Float
  let rotationScale: Float

  init(
    sourceType: SourceType = .arkit,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64,
    positionScale: Float,
    rotationScale: Float
  ) {
    self.sourceType = sourceType
    self.sequence = sequence
    self.sessionID = sessionID
    self.clientTimeNs = clientTimeNs
    self.positionScale = positionScale
    self.rotationScale = rotationScale
  }
}

struct EStopResetRequestPacket: Equatable, Sendable {
  let sourceType: SourceType
  let sequence: UInt32
  let sessionID: UInt32
  let clientTimeNs: UInt64

  init(
    sourceType: SourceType = .arkit,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64
  ) {
    self.sourceType = sourceType
    self.sequence = sequence
    self.sessionID = sessionID
    self.clientTimeNs = clientTimeNs
  }
}

struct EStopResetAckPacket: Equatable, Sendable {
  let sourceType: SourceType
  let sequence: UInt32
  let sessionID: UInt32
  let clientTimeNs: UInt64

  init(
    sourceType: SourceType = .arkit,
    sequence: UInt32,
    sessionID: UInt32,
    clientTimeNs: UInt64
  ) {
    self.sourceType = sourceType
    self.sequence = sequence
    self.sessionID = sessionID
    self.clientTimeNs = clientTimeNs
  }
}

enum TeleopPacket: Equatable, Sendable {
  case pose(PosePacket)
  case ping(PingPacket)
  case pong(PongPacket)
  case settings(SettingsPacket)
  case settingsAck(SettingsAckPacket)
  case estopResetRequest(EStopResetRequestPacket)
  case estopResetAck(EStopResetAckPacket)
}

enum PacketCodecError: String, Error, Equatable {
  case length
  case magic
  case version
  case size
  case type
  case source
  case reserved
  case flags
  case fieldRange = "field_range"
  case nonFinite = "non_finite"
  case quaternion
  case tracking
  case gripper
}
