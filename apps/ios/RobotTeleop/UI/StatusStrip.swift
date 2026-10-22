import SwiftUI

struct StatusStrip: View {
  @ObservedObject var model: AppModel

  private let columns = Array(
    repeating: GridItem(.flexible(minimum: 0), spacing: 8),
    count: 3
  )

  var body: some View {
    LazyVGrid(columns: columns, spacing: 8) {
      MetricCell(
        label: "追踪",
        value: trackingLabel,
        icon: "viewfinder",
        accent: trackingColor,
        identifier: "status.tracking",
        highlighted: model.status.trackingQuality != .normal
      )
      MetricCell(
        label: "发送 Hz",
        value: fpsLabel,
        icon: "speedometer",
        accent: TeleopTheme.accent,
        identifier: "status.send-fps"
      )
      MetricCell(
        label: "帧号",
        value: sequenceLabel,
        icon: "number",
        accent: TeleopTheme.accent,
        identifier: "status.sequence"
      )
      MetricCell(
        label: "往返延迟",
        value: rttLabel,
        icon: "arrow.left.arrow.right",
        accent: rttColor,
        identifier: "status.rtt"
      )
      MetricCell(
        label: "会话",
        value: sessionLabel,
        icon: "number.square",
        accent: TeleopTheme.warning,
        identifier: "status.session"
      )
      MetricCell(
        label: "UDP 链路",
        value: networkLabel,
        icon: "network",
        accent: networkColor,
        identifier: "status.network"
      )
    }
  }

  private var trackingLabel: String {
    switch model.status.trackingQuality {
    case .normal: "正常"
    case .limited: "受限"
    case .unavailable: "无追踪"
    }
  }

  private var trackingColor: Color {
    switch model.status.trackingQuality {
    case .normal: TeleopTheme.ready
    case .limited: TeleopTheme.warning
    case .unavailable: TeleopTheme.danger
    }
  }

  private var fpsLabel: String {
    model.status.sendRateHz > 0 ? String(format: "%.1f", model.status.sendRateHz) : "--"
  }

  private var sequenceLabel: String {
    model.status.lastSequence.map(String.init) ?? "--"
  }

  private var rttLabel: String {
    guard let rtt = model.status.rttMs else { return "--" }
    return rtt < 0.1 ? String(format: "%.2f ms", rtt) : String(format: "%.1f ms", rtt)
  }

  private var rttColor: Color {
    guard let rtt = model.status.rttMs else { return TeleopTheme.secondaryText }
    if rtt <= 20 { return TeleopTheme.ready }
    if rtt <= 50 { return TeleopTheme.warning }
    return TeleopTheme.danger
  }

  private var sessionLabel: String {
    model.status.sessionID == 0 ? "--" : String(format: "%08X", model.status.sessionID)
  }

  private var networkLabel: String {
    switch model.status.networkState {
    case .disconnected: "未连接"
    case .preparing: "连接中"
    case .ready: "就绪"
    case .waiting: "等待中"
    case .failed: "失败"
    }
  }

  private var networkColor: Color {
    switch model.status.networkState {
    case .ready: TeleopTheme.ready
    case .preparing, .waiting: TeleopTheme.warning
    case .failed: TeleopTheme.danger
    case .disconnected: TeleopTheme.secondaryText
    }
  }
}

private struct MetricCell: View {
  let label: String
  let value: String
  let icon: String
  let accent: Color
  let identifier: String
  var highlighted = false

  var body: some View {
    VStack(alignment: .leading, spacing: 5) {
      HStack(spacing: 5) {
        Image(systemName: icon)
          .font(.caption2.weight(.semibold))
        Text(label.uppercased())
          .font(.caption2.weight(.semibold))
          .lineLimit(1)
      }
      .foregroundStyle(accent)
      Text(value)
        .font(.subheadline.monospacedDigit().weight(.semibold))
        .foregroundStyle(TeleopTheme.primaryText)
        .lineLimit(1)
        .minimumScaleFactor(0.62)
        .frame(maxWidth: .infinity, alignment: .leading)
    }
    .padding(.horizontal, 10)
    .frame(maxWidth: .infinity, minHeight: 52, alignment: .leading)
    .background(highlighted ? accent.opacity(0.28) : TeleopTheme.surface)
    .overlay {
      RoundedRectangle(cornerRadius: 12)
        .stroke(highlighted ? accent : TeleopTheme.border, lineWidth: highlighted ? 2 : 1)
    }
    .clipShape(RoundedRectangle(cornerRadius: 12))
    .accessibilityElement(children: .ignore)
    .accessibilityLabel(label)
    .accessibilityValue(value)
    .accessibilityIdentifier(identifier)
  }
}
