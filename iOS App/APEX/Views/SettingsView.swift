import SwiftUI

@MainActor
struct SettingsView: View {
    @Environment(RobotStore.self) private var store
    @Environment(\.openURL) private var openURL
    @State private var hostText = ""
    @State private var robotIcon = false
    private static let robotIconName = "AppIconRobot"
    @FocusState private var hostFocused: Bool

    var body: some View {
        ApexScreen("Settings") {
            ApexCard("Robot Address", info: """
                With apex.local (the default) the app also tries robodog.local, the Pi's hostname, \
                and remembers whichever one answers. The Connection card shows the address in use. \
                The dashboard answers on port 80 when it can bind it, and always on port 5000, so \
                try apex.local:5000 if the plain name does not connect. Any other address you type, \
                such as an IP, is used exactly as entered.
                """) {
                TextField(RobotStore.defaultHost, text: $hostText)
                    .keyboardType(.URL)
                    .textContentType(.URL)
                    .autocorrectionDisabled()
                    .textInputAutocapitalization(.never)
                    .submitLabel(.done)
                    .focused($hostFocused)
                    .onSubmit(save)
                    .font(.system(size: 15, design: .monospaced))
                    .foregroundStyle(Color.apexText)
                    .padding(.horizontal, 10)
                    .frame(height: 44)
                    .background(Color.apexWell, in: RoundedRectangle(cornerRadius: 8))
                    .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(Color.apexLine, lineWidth: 1))

                HStack(spacing: 8) {
                    Button("apex.local") { hostText = "apex.local"; save() }
                        .apexButton(store.host == "apex.local" ? .on : .normal)
                    Button(":5000") { hostText = "apex.local:5000"; save() }
                        .apexButton(store.host == "apex.local:5000" ? .on : .normal)
                    Button("Save") { save() }
                        .apexButton(.primary)
                        .disabled(hostText.trimmingCharacters(in: .whitespaces) == store.host)
                }
            }

            ApexCard("Connection") {
                VStack(spacing: 8) {
                    infoRow("State", stateText)
                    infoRow("Server", store.client?.baseURL.absoluteString ?? "invalid")
                    infoRow("Last update", lastUpdateText)
                    if let err = store.lastError, store.connection != .connected {
                        infoRow("Last error", err)
                    }
                }
                Button {
                    Task { await store.refresh() }
                } label: {
                    Label("Retry now", systemImage: "arrow.clockwise")
                }
                .apexButton()
            }

            ApexCard("App Icon", info: "Switches the home screen icon between the A.P.E.X. logo and the robot render. iOS shows its own confirmation when the icon changes.") {
                Toggle(isOn: Binding(
                    get: { robotIcon },
                    set: { setRobotIcon($0) }
                )) {
                    Text("Robot render icon")
                        .font(.system(size: 14, weight: .semibold))
                        .foregroundStyle(Color.apexText)
                }
                .tint(Color.apexRed)
                .disabled(!UIApplication.shared.supportsAlternateIcons)
            }

            ApexCard("Web Dashboard", info: "Opens the original Mission Control page in your browser. Handy for anything this app does not cover yet.") {
                Button {
                    if let url = store.client?.baseURL { openURL(url) }
                } label: {
                    Label("Open in Browser", systemImage: "globe")
                }
                .apexButton()
                .disabled(store.client == nil)
            }

            ApexCard("About") {
                Text("APEX Mission Control for iOS. Talks to stream_server.py on the robot's Pi 5 over HTTP: polls /status once a second, streams /video_feed, and sends the same commands as the web dashboard.")
                    .font(.system(size: 13))
                    .foregroundStyle(Color.apexMuted)
                    .fixedSize(horizontal: false, vertical: true)
                infoRow("Version", appVersion)
            }
        }
        .onAppear {
            hostText = store.host
            robotIcon = UIApplication.shared.alternateIconName == Self.robotIconName
        }
    }

    private func setRobotIcon(_ on: Bool) {
        let previous = robotIcon
        robotIcon = on
        Task { @MainActor in
            do {
                try await UIApplication.shared.setAlternateIconName(on ? Self.robotIconName : nil)
            } catch {
                robotIcon = previous
                store.showToast("Could not change the icon: \(error.localizedDescription)")
            }
        }
    }

    private func save() {
        hostFocused = false
        store.setHost(hostText)
        hostText = store.host
    }

    private var stateText: String {
        switch store.connection {
        case .connected: return "Connected"
        case .connecting: return "Connecting"
        case .disconnected: return "Not reachable"
        }
    }

    private var lastUpdateText: String {
        guard let d = store.lastUpdate else { return "never" }
        return d.formatted(date: .omitted, time: .standard)
    }

    private var appVersion: String {
        let v = Bundle.main.object(forInfoDictionaryKey: "CFBundleShortVersionString") as? String ?? "?"
        let b = Bundle.main.object(forInfoDictionaryKey: "CFBundleVersion") as? String ?? "?"
        return "\(v) (\(b))"
    }

    private func infoRow(_ label: String, _ value: String) -> some View {
        HStack(alignment: .top) {
            Text(label.uppercased())
                .tracking(0.8)
                .foregroundStyle(Color.apexMuted)
            Spacer(minLength: 12)
            Text(value)
                .foregroundStyle(Color.apexText)
                .multilineTextAlignment(.trailing)
                .textSelection(.enabled)
        }
        .font(.system(size: 13, design: .monospaced))
    }
}
