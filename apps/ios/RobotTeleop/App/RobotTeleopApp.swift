import SwiftUI

@main
struct RobotTeleopApp: App {
  @StateObject private var model = AppModel()
  @Environment(\.scenePhase) private var scenePhase

  var body: some Scene {
    WindowGroup {
      ControlView(model: model)
        .preferredColorScheme(.dark)
        .onDisappear { model.stop() }
        .onChange(of: scenePhase) { _, phase in
          switch phase {
          case .active:
            break
          case .inactive:
            model.sceneDidBecomeInactive()
          case .background:
            model.sceneDidEnterBackground()
          @unknown default:
            model.sceneDidBecomeInactive()
          }
        }
    }
  }
}
