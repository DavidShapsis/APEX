import SwiftUI

@main
@MainActor
struct APEXApp: App {
    @State private var store = RobotStore()
    @Environment(\.scenePhase) private var scenePhase

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environment(store)
                .preferredColorScheme(.dark)
                .tint(Color.apexRed)
        }
        .onChange(of: scenePhase, initial: true) { _, phase in
            // Poll and stream only while the app is on screen.
            store.setActive(phase == .active)
        }
    }
}
