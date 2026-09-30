import SwiftUI

struct ControlView: View {
  @ObservedObject var model: AppModel
  @Environment(\.dynamicTypeSize) private var dynamicTypeSize

  var body: some View {
    GeometryReader { geometry in
      Group {
        if useLandscapeLayout(geometry.size) {
          landscapeLayout
        } else {
          portraitLayout
        }
      }
      .padding(.horizontal, 14)
      .padding(.vertical, 10)
      .frame(maxWidth: .infinity, maxHeight: .infinity)
    }
    .background(TeleopTheme.background.ignoresSafeArea())
  }

  private var portraitLayout: some View {
    VStack(spacing: 10) {
      EndpointHeader(model: model, compact: false)
      StatusStrip(model: model)
      ScrollView {
        VStack(spacing: 10) {
          TuningControls(model: model, compact: false)
          ControlActionRow(model: model)
        }
      }
      .scrollIndicators(.hidden)
      HoldControlSurface(model: model, compact: false)
        .frame(height: 122)
    }
  }

  private var landscapeLayout: some View {
    VStack(spacing: 8) {
      EndpointHeader(model: model, compact: true)
      HStack(alignment: .top, spacing: 12) {
        ScrollView {
          VStack(spacing: 10) {
            StatusStrip(model: model)
            TuningControls(model: model, compact: true)
          }
        }
        .scrollIndicators(.hidden)
        .frame(maxWidth: .infinity)

        VStack(spacing: 10) {
          ControlActionRow(model: model)
          HoldControlSurface(model: model, compact: true)
        }
        .frame(width: 390)
        .frame(maxHeight: .infinity)
      }
    }
  }

  private func useLandscapeLayout(_ size: CGSize) -> Bool {
    size.width > size.height && size.width >= 760 && !dynamicTypeSize.isAccessibilitySize
  }
}
