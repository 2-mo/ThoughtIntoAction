import Foundation

enum ClockSampleError: Error, Equatable {
  case clientChronology
  case serverChronology
  case negativeRoundTrip
}

struct ClockSample: Equatable, Sendable {
  let clientSendTimeNs: UInt64
  let serverReceiveTimeNs: UInt64
  let serverSendTimeNs: UInt64
  let clientReceiveTimeNs: UInt64
  let rttNs: UInt64
  let serverMinusClientNs: Double

  static func calculate(
    clientSendTimeNs: UInt64,
    serverReceiveTimeNs: UInt64,
    serverSendTimeNs: UInt64,
    clientReceiveTimeNs: UInt64
  ) throws -> ClockSample {
    guard clientReceiveTimeNs >= clientSendTimeNs else {
      throw ClockSampleError.clientChronology
    }
    guard serverSendTimeNs >= serverReceiveTimeNs else {
      throw ClockSampleError.serverChronology
    }

    let observedRoundTrip = clientReceiveTimeNs - clientSendTimeNs
    let serverProcessing = serverSendTimeNs - serverReceiveTimeNs
    guard observedRoundTrip >= serverProcessing else {
      throw ClockSampleError.negativeRoundTrip
    }
    let rttNs = observedRoundTrip - serverProcessing
    let offset =
      (signedDifference(serverReceiveTimeNs, clientSendTimeNs)
        + signedDifference(serverSendTimeNs, clientReceiveTimeNs)) / 2

    return ClockSample(
      clientSendTimeNs: clientSendTimeNs,
      serverReceiveTimeNs: serverReceiveTimeNs,
      serverSendTimeNs: serverSendTimeNs,
      clientReceiveTimeNs: clientReceiveTimeNs,
      rttNs: rttNs,
      serverMinusClientNs: offset
    )
  }

  private static func signedDifference(_ lhs: UInt64, _ rhs: UInt64) -> Double {
    if lhs >= rhs {
      return Double(lhs - rhs)
    }
    return -Double(rhs - lhs)
  }
}

final class ClockSyncEstimator: @unchecked Sendable {
  private let maximumRTTNs: UInt64
  private let capacity: Int
  private let lowRTTSampleCount: Int
  private let lock = NSLock()
  private var samples: [ClockSample] = []
  private var rejectedSamples = 0

  init(maximumRTTNs: UInt64, capacity: Int, lowRTTSampleCount: Int) {
    precondition(maximumRTTNs > 0)
    precondition(capacity > 0)
    precondition(lowRTTSampleCount > 0 && lowRTTSampleCount <= capacity)
    self.maximumRTTNs = maximumRTTNs
    self.capacity = capacity
    self.lowRTTSampleCount = lowRTTSampleCount
  }

  convenience init() {
    self.init(maximumRTTNs: 100_000_000, capacity: 32, lowRTTSampleCount: 8)
  }

  @discardableResult
  func add(_ sample: ClockSample) -> Bool {
    lock.withLock {
      guard sample.rttNs <= maximumRTTNs else {
        rejectedSamples += 1
        return false
      }
      if samples.count == capacity {
        samples.removeFirst()
      }
      samples.append(sample)
      return true
    }
  }

  @discardableResult
  func add(
    clientSendTimeNs: UInt64,
    serverReceiveTimeNs: UInt64,
    serverSendTimeNs: UInt64,
    clientReceiveTimeNs: UInt64
  ) -> Bool {
    do {
      return add(
        try ClockSample.calculate(
          clientSendTimeNs: clientSendTimeNs,
          serverReceiveTimeNs: serverReceiveTimeNs,
          serverSendTimeNs: serverSendTimeNs,
          clientReceiveTimeNs: clientReceiveTimeNs
        )
      )
    } catch {
      lock.withLock {
        rejectedSamples += 1
      }
      return false
    }
  }

  func reset() {
    lock.withLock {
      samples.removeAll(keepingCapacity: true)
      rejectedSamples = 0
    }
  }

  var sampleCount: Int {
    lock.withLock { samples.count }
  }

  var rejectedSampleCount: Int {
    lock.withLock { rejectedSamples }
  }

  var minimumRTTNs: UInt64? {
    lock.withLock { samples.map(\.rttNs).min() }
  }

  var offsetNs: Double? {
    lock.withLock {
      let selected = samples.sorted { $0.rttNs < $1.rttNs }.prefix(lowRTTSampleCount)
      let offsets = selected.map(\.serverMinusClientNs).sorted()
      guard !offsets.isEmpty else {
        return nil
      }
      let middle = offsets.count / 2
      if offsets.count.isMultiple(of: 2) {
        return (offsets[middle - 1] + offsets[middle]) / 2
      }
      return offsets[middle]
    }
  }
}
