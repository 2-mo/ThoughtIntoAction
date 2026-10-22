import SwiftUI

struct ControlView: View {
  @ObservedObject var model: AppModel
  @Environment(\.dynamicTypeSize) private var dynamicTypeSize

  var body: some View {
    GeometryReader { geometry in
      Group {
        if geometry.size.width > geometry.size.height && geometry.size.width >= 700 && !dynamicTypeSize.isAccessibilitySize {
          landscapeLayout(geometry.size)
        } else {
          portraitLayout(geometry.size)
        }
      }
      .padding(.horizontal, 14).padding(.vertical, 10)
      .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
    .background(TeleopTheme.background.ignoresSafeArea())
  }

  private var dashboard: some View {
    VStack(spacing: 10) {
      StatusStrip(model: model)
      ControlStatusCard(model: model)
      TuningControls(model: model)
    }
  }

  private func portraitLayout(_ size: CGSize) -> some View {
    VStack(spacing: 10) {
      EndpointHeader(model: model)
      if size.height < 660 || dynamicTypeSize.isAccessibilitySize {
        // Only short screens / accessibility text need scrolling. Controls stay outside.
        ScrollView { dashboard }.scrollIndicators(.visible)
      } else {
        dashboard.fixedSize(horizontal: false, vertical: true)
      }
      ControlActionRow(model: model)
      HoldControlSurface(model: model, compact: false)
        .frame(minHeight: min(240, max(160, size.height - 510)), maxHeight: .infinity)
      ControlUtilityRow(model: model)
    }
  }

  private func landscapeLayout(_ size: CGSize) -> some View {
    VStack(spacing: 8) {
      EndpointHeader(model: model)
      HStack(alignment: .top, spacing: 14) {
        VStack(spacing: 8) {
          ScrollView { dashboard }.scrollIndicators(.visible)
          ControlUtilityRow(model: model)
        }.frame(maxWidth: .infinity)
        VStack(spacing: 8) {
          ControlActionRow(model: model)
          HoldControlSurface(model: model, compact: true)
        }
        .frame(width: size.width * 0.53, height: max(0, size.height - 72))
      }
    }
  }
}
