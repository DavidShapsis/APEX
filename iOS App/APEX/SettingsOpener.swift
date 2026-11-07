import UIKit

/// Opens the iOS Wi-Fi settings page through the private App-Prefs URL (not App Store safe,
/// fine for sideloading). Tries the Wi-Fi page, then the top of Settings, then this app's own
/// Settings page. Where it lands depends on the iOS version.
@MainActor
enum SettingsOpener {
    private static let candidates = ["App-Prefs:root=WIFI", "App-Prefs:WIFI", "prefs:root=WIFI", "App-Prefs:"]

    static func openWiFi() {
        attempt(0)
    }

    private static func attempt(_ index: Int) {
        if index < candidates.count, let url = URL(string: candidates[index]) {
            UIApplication.shared.open(url, options: [:]) { opened in
                if !opened {
                    Task { @MainActor in attempt(index + 1) }
                }
            }
        } else if let own = URL(string: UIApplication.openSettingsURLString) {
            UIApplication.shared.open(own)
        }
    }
}
