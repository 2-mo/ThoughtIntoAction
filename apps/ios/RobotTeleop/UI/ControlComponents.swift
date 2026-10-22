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
  @State private var settingsOpen = false

  var body: some View {
    HStack(spacing: 10) {
      Image(systemName: "move.3d")
        .font(.system(size: 24, weight: .semibold))
        .foregroundStyle(TeleopTheme.accent)
      VStack(alignment: .leading, spacing: 3) {
        Text("知行有灵").font(.headline).foregroundStyle(TeleopTheme.primaryText)
        Text("手机遥操 · RobotTeleop").font(.caption2).foregroundStyle(TeleopTheme.secondaryText)
      }
      Spacer(minLength: 6)
      Button { settingsOpen = true } label: {
        VStack(alignment: .trailing, spacing: 4) {
          Label("连接设置", systemImage: "network").font(.caption.weight(.semibold))
          Text(model.host).font(.caption2.monospaced()).lineLimit(1)
        }
        .frame(minHeight: 44)
      }
      .buttonStyle(.plain).foregroundStyle(TeleopTheme.secondaryText)
      .disabled(model.clutchActive)
      .accessibilityIdentifier("connection.settings")
    }
    .sheet(isPresented: $settingsOpen) {
      NavigationStack {
        Form {
          Section("电脑接收服务") {
            TextField("服务器地址", text: $model.host)
              .textInputAutocapitalization(.never).autocorrectionDisabled()
              .keyboardType(.numbersAndPunctuation).disabled(!model.endpointEditable)
            TextField("UDP 端口", text: $model.port)
              .keyboardType(.numberPad).disabled(!model.endpointEditable)
            Button(model.connectionOpen ? "断开连接" : "连接") { model.performConnectionAction() }
              .accessibilityIdentifier("connection.toggle")
            Text("更换地址前先断开连接。断开也会停止手机遥操。")
              .font(.caption).foregroundStyle(.secondary)
          }
          Section("采集与连接详情") {
            LabeledContent("姿态来源", value: model.providerKind == .simulated ? "模拟姿态" : "ARKit")
            if let capture = model.captureSummary { Text(capture).font(.caption.monospaced()) }
            if let message = model.endpointMessage { Text(message).foregroundStyle(TeleopTheme.warning) }
          }
        }
        .navigationTitle("连接设置").navigationBarTitleDisplayMode(.inline)
        .toolbar { ToolbarItem(placement: .confirmationAction) { Button("完成") { settingsOpen = false } } }
      }
      .preferredColorScheme(.dark)
      .presentationDetents([.medium, .large]).presentationDragIndicator(.visible)
    }
  }
}

struct ControlStatusCard: View {
  @ObservedObject var model: AppModel

  private var color: Color {
    if model.stopLatched || model.status.trackingQuality == .unavailable { return TeleopTheme.danger }
    if model.status.trackingQuality != .normal || !model.status.controlAvailable { return TeleopTheme.warning }
    return model.status.phoneControl?.enabled == true ? TeleopTheme.accent : TeleopTheme.secondaryText
  }

  private var title: String {
    if model.stopLatched { return "停止已锁定" }
    if model.status.trackingQuality != .normal { return "等待追踪恢复" }
    if !model.status.controlAvailable { return "等待电脑确认" }
    if model.presetBusy { return "预设动作执行中" }
    if model.clutchActive { return "正在跟随" }
    return model.status.phoneControl?.enabled == true ? "遥操已开启 · 松手保持" : "仅监看 · 尚未接管"
  }

  var body: some View {
    HStack(spacing: 10) {
      Image(systemName: model.stopLatched ? "lock.fill" : "dot.radiowaves.left.and.right")
        .font(.title3).foregroundStyle(color)
      VStack(alignment: .leading, spacing: 3) {
        Text(title).font(.subheadline.weight(.semibold)).foregroundStyle(color)
        Text(model.status.trackingReason ?? model.controlSummary)
          .font(.caption2).foregroundStyle(TeleopTheme.secondaryText).lineLimit(2)
      }
      Spacer(minLength: 0)
    }
    .padding(.horizontal, 12).padding(.vertical, 8)
    .frame(maxWidth: .infinity, alignment: .leading)
    .background(color.opacity(0.09), in: RoundedRectangle(cornerRadius: 12))
    .overlay { RoundedRectangle(cornerRadius: 12).stroke(color.opacity(0.35), lineWidth: 1) }
    .accessibilityElement(children: .combine).accessibilityIdentifier("control.status")
  }
}

struct TuningControls: View {
  @ObservedObject var model: AppModel
  @State private var settingsOpen = false

  private var syncLabel: String {
    model.status.settingsState == .acknowledged ? "已同步" : model.status.settingsState == .pending ? "同步中" : "本地"
  }

  var body: some View {
    Button { settingsOpen = true } label: {
      HStack(spacing: 8) {
        Image(systemName: "slider.horizontal.3")
        Text("映射").fontWeight(.semibold)
        Text(String(format: "位移 %.2f× · 旋转 %.2f×", model.positionScale, model.rotationScale))
          .font(.caption.monospacedDigit()).lineLimit(1).minimumScaleFactor(0.8)
        Spacer(minLength: 0)
        settingsState
        Image(systemName: "chevron.right").font(.caption2)
      }
      .font(.caption).foregroundStyle(TeleopTheme.primaryText)
      .padding(.horizontal, 12).frame(minHeight: 44)
      .background(TeleopTheme.surface, in: RoundedRectangle(cornerRadius: 12))
    }
    .buttonStyle(.plain).disabled(!model.tuningEnabled || model.presetBusy)
    .accessibilityIdentifier("mapping.settings")
    .accessibilityValue(syncLabel)
    .sheet(isPresented: $settingsOpen) {
      NavigationStack {
        Form {
          Section("相对映射倍率") {
            SliderRow(title: "位移", valueText: String(format: "%.2f×", model.positionScale),
              value: Binding(get: { model.positionScale }, set: { model.setPositionScale($0) }),
              range: 0...Double(ControlDefaults.maximumPositionScale), step: 0.05,
              tint: TeleopTheme.accent, enabled: model.tuningEnabled, compact: false,
              onEditingEnded: { model.commitMappingScales() })
            SliderRow(title: "旋转", valueText: String(format: "%.2f×", model.rotationScale),
              value: Binding(get: { model.rotationScale }, set: { model.setRotationScale($0) }),
              range: 0...Double(ControlDefaults.maximumRotationScale), step: 0.05,
              tint: TeleopTheme.accent, enabled: model.tuningEnabled, compact: false,
              onEditingEnded: { model.commitMappingScales() })
            Text("1× 表示等比例跟随，松开后在下一次按住时生效。")
              .font(.caption).foregroundStyle(.secondary)
          }
          Section("空间方向") {
            Text("在预抓取位，竖直握持手机、后置相机朝向操作前方，再按住 Clutch 建立参考。松开重握保持空间方向；需要重新标定时点“方向对齐”。")
              .font(.subheadline)
          }
        }
        .navigationTitle("映射设置").navigationBarTitleDisplayMode(.inline)
        .toolbar { ToolbarItem(placement: .confirmationAction) { Button("完成") { settingsOpen = false } } }
      }
      .preferredColorScheme(.dark)
      .presentationDetents([.medium, .large]).presentationDragIndicator(.visible)
    }
  }

  private var settingsState: some View {
    Text(syncLabel).font(.caption2).foregroundStyle(model.status.settingsState == .acknowledged ? TeleopTheme.ready : TeleopTheme.warning)
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
    VStack(spacing: 8) {
      HStack(spacing: 8) {
        Button { model.startControl() } label: {
          Label("开始遥操", systemImage: "play.fill").frame(maxWidth: .infinity, minHeight: 46)
        }
        .buttonStyle(TeleopActionButtonStyle(foreground: TeleopTheme.background,
          background: TeleopTheme.accent, border: TeleopTheme.accent))
        .disabled(!model.canStartControl)
        Button { model.stopControl() } label: {
          Label("停止遥操", systemImage: "stop.fill").frame(maxWidth: .infinity, minHeight: 46)
        }
        .buttonStyle(TeleopActionButtonStyle(foreground: .white,
          background: TeleopTheme.danger, border: TeleopTheme.danger))
        .disabled(model.status.networkState != .ready)
      }
      HStack(spacing: 8) {
        Button { model.requestHome() } label: {
          Label("回零", systemImage: "house").frame(maxWidth: .infinity, minHeight: 44)
        }.disabled(!model.canHome)
          .accessibilityHint("先张开夹爪，确认松开后机械臂回零")
        Button { model.requestPregrasp() } label: {
          Label("预抓取", systemImage: "scope").frame(maxWidth: .infinity, minHeight: 44)
        }.disabled(!model.canHome)
        Button { model.requestAlignment() } label: {
          Label("方向对齐", systemImage: "move.3d").frame(maxWidth: .infinity, minHeight: 44)
        }.disabled(!model.canHome)
          .accessibilityHint("松开后重新摆正手机，下次按住 Clutch 建立方向参考，不移动机械臂")
      }
      .buttonStyle(TeleopActionButtonStyle(foreground: TeleopTheme.primaryText,
        background: TeleopTheme.raised, border: TeleopTheme.border))
    }
  }
}

struct ControlUtilityRow: View {
  @ObservedObject var model: AppModel

  var body: some View {
    HStack(spacing: 8) {
      Button { model.recoverControl() } label: {
        Label(model.recoverControlLabel, systemImage: "arrow.clockwise")
          .frame(maxWidth: .infinity, minHeight: 44)
      }
      .disabled(!model.canRecoverControl)
      .accessibilityHint("恢复连接或清除手机停止锁定，随后手动开始遥操")
      Button { model.latchEstop() } label: {
        Label("停止锁定", systemImage: "lock.fill").frame(maxWidth: .infinity, minHeight: 44)
      }.disabled(model.stopLatched)
    }
    .buttonStyle(TeleopActionButtonStyle(foreground: TeleopTheme.secondaryText,
      background: TeleopTheme.surface, border: TeleopTheme.border))
  }
}

private struct TeleopActionButtonStyle: ButtonStyle {
  @Environment(\.isEnabled) private var isEnabled
  let foreground: Color
  let background: Color
  let border: Color

  func makeBody(configuration: Configuration) -> some View {
    configuration.label
      .font(.subheadline.weight(.semibold))
      .foregroundStyle(foreground)
      .background(configuration.isPressed ? background.opacity(0.72) : background)
      .overlay {
        RoundedRectangle(cornerRadius: 12)
          .stroke(border, lineWidth: 1)
      }
      .clipShape(RoundedRectangle(cornerRadius: 12))
      .opacity(isEnabled ? (configuration.isPressed ? 0.9 : 1) : 0.4)
  }
}

struct HoldControlSurface: View {
  @ObservedObject var model: AppModel
  let compact: Bool

  @State private var gestureActive = false
  @State private var gestureBlocked = false
  @State private var gripperAtTouch = 0.0
  @GestureState private var touching = false

  var body: some View {
    VStack(spacing: compact ? 8 : 12) {
      HStack {
        Text("CLUTCH · 跟随").font(.caption.weight(.semibold))
        Spacer()
        Text(String(format: "夹爪 %.0f%%", model.gripper * 100)).font(.caption.monospacedDigit())
      }
      Spacer(minLength: 0)
      if compact {
        HStack(spacing: 12) {
          Image(systemName: holdIcon).font(.system(size: 26, weight: .medium))
          holdInstructions
        }
      } else {
        Image(systemName: holdIcon).font(.system(size: 34, weight: .medium))
        holdInstructions
      }
      Spacer(minLength: 0)
      HStack {
        Label("上滑 · 张开", systemImage: "arrow.up")
        Spacer()
        Label("下滑 · 闭合", systemImage: "arrow.down")
      }.font(.caption.weight(.medium))
      ProgressView(value: model.gripper).tint(holdForeground).accessibilityHidden(true)
    }
    .padding(compact ? 12 : 16)
    .foregroundStyle(holdForeground)
    .frame(maxWidth: .infinity, maxHeight: .infinity)
    .frame(minHeight: compact ? 148 : 160)
    .background(holdBackground, in: RoundedRectangle(cornerRadius: 20))
    .overlay { RoundedRectangle(cornerRadius: 20).stroke(holdBorder, lineWidth: model.clutchActive ? 2 : 1) }
    .contentShape(Rectangle())
    .gesture(
      DragGesture(minimumDistance: 0)
        .updating($touching) { _, state, _ in state = true }
        .onChanged { value in
          guard model.canControl, !gestureBlocked else { gestureBlocked = true; return }
          if !gestureActive {
            gestureActive = true
            gripperAtTouch = model.gripper
            model.setClutchActive(true)
          }
          model.setGripper(gripperAtTouch - Double(value.translation.height) / 160.0)
        }
        .onEnded { _ in
          gestureActive = false
          gestureBlocked = false
          model.setClutchActive(false)
        }
    )
    .onChange(of: touching) { _, down in
      if !down {
        gestureActive = false
        gestureBlocked = false
        model.setClutchActive(false)
      }
    }
    .onChange(of: model.canControl) { _, canControl in
      if !canControl {
        if gestureActive { gestureBlocked = true }
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
    .accessibilityLabel("按住跟随，上滑开夹爪，下滑闭夹爪")
    .accessibilityIdentifier("control.clutch")
    .accessibilityValue(holdState)
  }

  private var holdInstructions: some View {
    VStack(spacing: 4) {
      Text(holdLabel).font(.system(size: compact ? 19 : 23, weight: .semibold))
        .lineLimit(1).minimumScaleFactor(0.7)
      Text(holdState).font(.caption).lineLimit(2).multilineTextAlignment(.center)
    }
  }

  private var holdLabel: String {
    if model.stopLatched { return "控制已锁定" }
    if model.presetBusy { return "预设动作执行中" }
    return model.clutchActive ? "正在跟随" : "按住跟随"
  }

  private var holdState: String {
    if model.stopLatched { return "点恢复解除手机锁定" }
    if model.presetBusy { return "可随时点停止遥操取消动作" }
    if !model.status.controlAvailable { return "等待电脑接收服务" }
    if model.status.trackingQuality != .normal { return "追踪未就绪" }
    if model.status.phoneControl?.enabled != true { return "先点开始遥操" }
    return String(format: "夹爪开度 %.0f%% · 松手保持", model.gripper * 100)
  }

  private var holdIcon: String {
    if model.stopLatched { return "lock.fill" }
    return model.clutchActive ? "hand.tap.fill" : "hand.tap"
  }

  private var holdForeground: Color {
    model.clutchActive ? TeleopTheme.background : model.canControl ? TeleopTheme.accent : TeleopTheme.secondaryText
  }

  private var holdBackground: Color {
    if model.stopLatched { return TeleopTheme.danger.opacity(0.24) }
    if model.clutchActive { return TeleopTheme.accent }
    if model.canControl { return TeleopTheme.accent.opacity(0.12) }
    return TeleopTheme.surface
  }

  private var holdBorder: Color {
    if model.stopLatched { return TeleopTheme.danger }
    if model.clutchActive { return TeleopTheme.ready }
    if model.canControl { return TeleopTheme.accent }
    return TeleopTheme.border
  }
}
