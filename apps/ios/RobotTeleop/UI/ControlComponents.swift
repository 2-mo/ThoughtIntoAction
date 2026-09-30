import SwiftUI

enum TeleopTheme {
  static let background = Color(red: 0.045, green: 0.052, blue: 0.060)
  static let surface = Color(red: 0.085, green: 0.096, blue: 0.106)
  static let raised = Color(red: 0.12, green: 0.133, blue: 0.145)
  static let border = Color(red: 0.22, green: 0.24, blue: 0.26)
  static let primaryText = Color(red: 0.94, green: 0.95, blue: 0.96)
  static let secondaryText = Color(red: 0.61, green: 0.65, blue: 0.68)
  static let accent = Color(red: 0.18, green: 0.61, blue: 0.86)
  static let ready = Color(red: 0.22, green: 0.78, blue: 0.44)
  static let warning = Color(red: 0.96, green: 0.70, blue: 0.20)
  static let danger = Color(red: 0.91, green: 0.22, blue: 0.24)
}

struct EndpointHeader: View {
  @ObservedObject var model: AppModel
  let compact: Bool

  @FocusState private var focusedField: Field?

  private enum Field {
    case host
    case port
  }

  var body: some View {
    VStack(spacing: 6) {
      if compact {
        HStack(spacing: 12) {
          brand
            .frame(width: 170, alignment: .leading)
          endpointFields
        }
      } else {
        brand
        endpointFields
      }
      Text(endpointStatusMessage ?? " ")
        .font(.caption2)
        .foregroundStyle(messageColor)
        .lineLimit(1)
        .frame(maxWidth: .infinity, minHeight: 14, alignment: .leading)
        .opacity(endpointStatusMessage == nil ? 0 : 1)
    }
  }

  private var brand: some View {
    HStack(spacing: 9) {
      Image(systemName: "move.3d")
        .font(.system(size: 19, weight: .semibold))
        .foregroundStyle(TeleopTheme.accent)
        .frame(width: 28, height: 28)
      VStack(alignment: .leading, spacing: 1) {
        Text("Robot Teleop")
          .font(.headline)
          .foregroundStyle(TeleopTheme.primaryText)
        Text(model.providerKind == .simulated ? "SIMULATED POSE" : "ARKIT POSE")
          .font(.caption2.weight(.semibold))
          .foregroundStyle(TeleopTheme.secondaryText)
      }
      Spacer(minLength: 8)
    }
  }

  private var endpointFields: some View {
    HStack(spacing: 8) {
      TextField("Ubuntu host", text: $model.host)
        .focused($focusedField, equals: .host)
        .textInputAutocapitalization(.never)
        .autocorrectionDisabled()
        .keyboardType(.numbersAndPunctuation)
        .submitLabel(.next)
        .onSubmit { focusedField = .port }
        .disabled(!model.endpointEditable)
        .accessibilityLabel("Ubuntu host")
        .frame(maxWidth: .infinity)
        .teleopTextField()

      TextField("Port", text: $model.port)
        .focused($focusedField, equals: .port)
        .keyboardType(.numberPad)
        .multilineTextAlignment(.center)
        .disabled(!model.endpointEditable)
        .accessibilityLabel("UDP port")
        .frame(width: 72)
        .teleopTextField()

      Button {
        focusedField = nil
        model.performConnectionAction()
      } label: {
        Label(connectionCommand, systemImage: connectionIcon)
          .font(.subheadline.weight(.semibold))
          .lineLimit(1)
          .frame(width: 120, height: 42)
      }
      .buttonStyle(.plain)
      .foregroundStyle(connectionForeground)
      .background(connectionBackground)
      .overlay {
        RoundedRectangle(cornerRadius: 6)
          .stroke(connectionBorder, lineWidth: 1)
      }
      .clipShape(RoundedRectangle(cornerRadius: 6))
      .accessibilityLabel(connectionCommand)
    }
  }

  private var connectionCommand: String {
    model.connectionOpen ? "Disconnect" : "Connect"
  }

  private var connectionIcon: String {
    model.connectionOpen ? "xmark" : "antenna.radiowaves.left.and.right"
  }

  private var connectionForeground: Color {
    model.connectionOpen ? TeleopTheme.primaryText : TeleopTheme.background
  }

  private var connectionBackground: Color {
    model.connectionOpen ? TeleopTheme.raised : TeleopTheme.ready
  }

  private var connectionBorder: Color {
    model.connectionOpen ? TeleopTheme.border : TeleopTheme.ready
  }

  private var messageColor: Color {
    switch model.status.networkState {
    case .waiting:
      TeleopTheme.warning
    case .failed:
      TeleopTheme.danger
    case .disconnected, .preparing, .ready:
      TeleopTheme.secondaryText
    }
  }

  private var endpointStatusMessage: String? {
    model.endpointMessage ?? model.captureSummary
  }
}

struct TuningControls: View {
  @ObservedObject var model: AppModel
  let compact: Bool

  var body: some View {
    VStack(spacing: 0) {
      HStack {
        Label("Controls", systemImage: "slider.horizontal.3")
          .font(.subheadline.weight(.semibold))
          .foregroundStyle(TeleopTheme.primaryText)
        Spacer()
        settingsState
      }
      .padding(.horizontal, 14)
      .frame(height: compact ? 32 : 38)

      Divider().overlay(TeleopTheme.border)

      SliderRow(
        title: "Gripper",
        valueText: String(format: "%.0f%%", model.gripper * 100),
        value: Binding(
          get: { model.gripper },
          set: { value in model.setGripper(value) }
        ),
        range: 0...1,
        step: 0.01,
        tint: TeleopTheme.warning,
        enabled: !model.estopLatched,
        compact: compact
      )

      Divider().overlay(TeleopTheme.border)

      SliderRow(
        title: "Position",
        valueText: String(format: "%.2fx", model.positionScale),
        value: Binding(
          get: { model.positionScale },
          set: { value in model.setPositionScale(value) }
        ),
        range: 0...Double(ControlDefaults.maximumPositionScale),
        step: 0.05,
        tint: TeleopTheme.accent,
        enabled: model.tuningEnabled,
        compact: compact,
        onEditingEnded: { model.commitMappingScales() }
      )

      Divider().overlay(TeleopTheme.border)

      SliderRow(
        title: "Rotation",
        valueText: String(format: "%.2fx", model.rotationScale),
        value: Binding(
          get: { model.rotationScale },
          set: { value in model.setRotationScale(value) }
        ),
        range: 0...Double(ControlDefaults.maximumRotationScale),
        step: 0.05,
        tint: TeleopTheme.accent,
        enabled: model.tuningEnabled,
        compact: compact,
        onEditingEnded: { model.commitMappingScales() }
      )
    }
    .background(TeleopTheme.surface)
    .overlay {
      RoundedRectangle(cornerRadius: 6)
        .stroke(TeleopTheme.border, lineWidth: 1)
    }
    .clipShape(RoundedRectangle(cornerRadius: 6))
  }

  private var settingsState: some View {
    Label(settingsLabel, systemImage: settingsIcon)
      .font(.caption2.weight(.semibold))
      .foregroundStyle(settingsColor)
      .lineLimit(1)
      .accessibilityLabel("Settings")
      .accessibilityValue(settingsLabel)
      .accessibilityIdentifier("status.settings")
  }

  private var settingsLabel: String {
    switch model.status.settingsState {
    case .notSent: "LOCAL"
    case .pending: "SYNCING"
    case .acknowledged: "SYNCED"
    }
  }

  private var settingsIcon: String {
    switch model.status.settingsState {
    case .notSent: "minus.circle"
    case .pending: "clock"
    case .acknowledged: "checkmark.circle.fill"
    }
  }

  private var settingsColor: Color {
    switch model.status.settingsState {
    case .notSent: TeleopTheme.secondaryText
    case .pending: TeleopTheme.warning
    case .acknowledged: TeleopTheme.ready
    }
  }
}

private struct SliderRow: View {
  let title: String
  let valueText: String
  @Binding var value: Double
  let range: ClosedRange<Double>
  let step: Double
  let tint: Color
  let enabled: Bool
  let compact: Bool
  var onEditingEnded: () -> Void = {}

  var body: some View {
    VStack(spacing: 4) {
      HStack {
        Text(title)
          .font(.caption)
          .foregroundStyle(TeleopTheme.secondaryText)
        Spacer()
        Text(valueText)
          .font(.caption.monospacedDigit().weight(.semibold))
          .foregroundStyle(TeleopTheme.primaryText)
          .frame(width: 58, alignment: .trailing)
      }
      Slider(
        value: $value,
        in: range,
        step: step,
        onEditingChanged: { editing in
          if !editing { onEditingEnded() }
        }
      )
      .tint(tint)
      .disabled(!enabled)
      .accessibilityLabel(title)
      .accessibilityValue(valueText)
    }
    .padding(.horizontal, 14)
    .padding(.vertical, compact ? 2 : 8)
    .opacity(enabled ? 1 : 0.45)
  }
}

struct ControlActionRow: View {
  @ObservedObject var model: AppModel

  var body: some View {
    HStack(spacing: 10) {
      Button {
        model.requestHome()
      } label: {
        Label("Home", systemImage: "house")
          .frame(maxWidth: .infinity, minHeight: 54)
      }
      .buttonStyle(
        TeleopActionButtonStyle(
          foreground: TeleopTheme.primaryText,
          background: TeleopTheme.raised,
          border: TeleopTheme.border
        )
      )
      .disabled(!model.canHome)

      Button {
        model.recoverControl()
      } label: {
        Label(model.recoverControlLabel, systemImage: "arrow.clockwise")
          .lineLimit(1)
          .minimumScaleFactor(0.75)
          .frame(maxWidth: .infinity, minHeight: 54)
      }
      .buttonStyle(
        TeleopActionButtonStyle(
          foreground: TeleopTheme.primaryText,
          background: TeleopTheme.raised,
          border: TeleopTheme.border
        )
      )
      .disabled(!model.canRecoverControl)
      .accessibilityHint("Releases clutch and automatically restores control")

      if !model.estopLatched {
        Button {
          model.latchEstop()
        } label: {
          Label("E-STOP", systemImage: "stop.circle.fill")
            .lineLimit(1)
            .minimumScaleFactor(0.75)
            .frame(maxWidth: .infinity, minHeight: 54)
        }
        .buttonStyle(
          TeleopActionButtonStyle(
            foreground: .white,
            background: TeleopTheme.danger,
            border: TeleopTheme.danger
          )
        )
        .accessibilityHint("Latches the software emergency stop request")
      }
    }
  }
}

private struct TeleopActionButtonStyle: ButtonStyle {
  let foreground: Color
  let background: Color
  let border: Color

  func makeBody(configuration: Configuration) -> some View {
    configuration.label
      .font(.subheadline.weight(.semibold))
      .foregroundStyle(foreground)
      .background(configuration.isPressed ? background.opacity(0.72) : background)
      .overlay {
        RoundedRectangle(cornerRadius: 6)
          .stroke(border, lineWidth: 1)
      }
      .clipShape(RoundedRectangle(cornerRadius: 6))
      .opacity(configuration.isPressed ? 0.9 : 1)
  }
}

struct HoldControlSurface: View {
  @ObservedObject var model: AppModel
  let compact: Bool

  @State private var gestureActive = false

  var body: some View {
    VStack(spacing: 7) {
      Image(systemName: holdIcon)
        .font(.system(size: compact ? 27 : 30, weight: .semibold))
      Text(holdLabel)
        .font(.headline.weight(.bold))
        .lineLimit(1)
        .minimumScaleFactor(0.75)
      Text(holdState)
        .font(.caption.monospacedDigit().weight(.medium))
        .lineLimit(1)
        .minimumScaleFactor(0.75)
    }
    .foregroundStyle(holdForeground)
    .frame(maxWidth: .infinity, maxHeight: .infinity)
    .frame(minHeight: compact ? 148 : 116)
    .background(holdBackground)
    .overlay {
      RoundedRectangle(cornerRadius: 8)
        .stroke(holdBorder, lineWidth: model.clutchActive ? 2 : 1)
    }
    .clipShape(RoundedRectangle(cornerRadius: 8))
    .contentShape(Rectangle())
    .gesture(
      DragGesture(minimumDistance: 0)
        .onChanged { _ in
          guard !gestureActive else { return }
          gestureActive = true
          model.setClutchActive(true)
        }
        .onEnded { _ in
          gestureActive = false
          model.setClutchActive(false)
        }
    )
    .onChange(of: model.canControl) { _, canControl in
      if !canControl {
        gestureActive = false
        model.setClutchActive(false)
      }
    }
    .onDisappear {
      gestureActive = false
      model.setClutchActive(false)
    }
    .accessibilityElement(children: .combine)
    .accessibilityAddTraits(.isButton)
    .accessibilityLabel("Hold to control")
    .accessibilityValue(holdState)
  }

  private var holdLabel: String {
    if model.estopLatched { return "CONTROL LOCKED" }
    return model.clutchActive ? "CONTROLLING" : "HOLD TO CONTROL"
  }

  private var holdState: String {
    if model.estopLatched { return "E-STOP LATCHED" }
    if model.status.networkState != .ready { return "NETWORK NOT READY" }
    if model.status.trackingQuality == .unavailable { return "TRACKING UNAVAILABLE" }
    return model.clutchActive ? "CLUTCH ACTIVE" : "READY"
  }

  private var holdIcon: String {
    if model.estopLatched { return "lock.fill" }
    return model.clutchActive ? "hand.tap.fill" : "hand.tap"
  }

  private var holdForeground: Color {
    model.canControl || model.clutchActive ? TeleopTheme.background : TeleopTheme.secondaryText
  }

  private var holdBackground: Color {
    if model.estopLatched { return TeleopTheme.danger.opacity(0.24) }
    if model.clutchActive { return TeleopTheme.ready }
    if model.canControl { return TeleopTheme.accent }
    return TeleopTheme.surface
  }

  private var holdBorder: Color {
    if model.estopLatched { return TeleopTheme.danger }
    if model.clutchActive { return TeleopTheme.ready }
    if model.canControl { return TeleopTheme.accent }
    return TeleopTheme.border
  }
}

private struct TeleopTextFieldModifier: ViewModifier {
  func body(content: Content) -> some View {
    content
      .font(.subheadline.monospaced())
      .foregroundStyle(TeleopTheme.primaryText)
      .padding(.horizontal, 11)
      .frame(height: 42)
      .background(TeleopTheme.surface)
      .overlay {
        RoundedRectangle(cornerRadius: 6)
          .stroke(TeleopTheme.border, lineWidth: 1)
      }
      .clipShape(RoundedRectangle(cornerRadius: 6))
  }
}

extension View {
  fileprivate func teleopTextField() -> some View {
    modifier(TeleopTextFieldModifier())
  }
}
