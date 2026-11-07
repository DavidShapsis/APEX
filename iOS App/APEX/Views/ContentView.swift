import SwiftUI

@MainActor
struct ContentView: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        TabView {
            DriveView()
                .tabItem { Label("Drive", systemImage: "gamecontroller.fill") }
            NavigateView()
                .tabItem { Label("Navigate", systemImage: "map.fill") }
            SystemsView()
                .tabItem { Label("Systems", systemImage: "waveform.path.ecg") }
            SettingsView()
                .tabItem { Label("Settings", systemImage: "gearshape.fill") }
        }
        .overlay(alignment: .top) {
            if let msg = store.toast {
                ToastView(text: msg, success: store.toastSuccess)
                    .padding(.horizontal, 16)
                    .padding(.top, 6)
                    .transition(.move(edge: .top).combined(with: .opacity))
                    .onTapGesture { store.toast = nil }
            }
        }
        .animation(.spring(duration: 0.3), value: store.toast)
    }
}

/// Shared scaffold for every tab: themed background, nav title with a
/// connection light, the "not connected" banner, then the cards.
@MainActor
struct ApexScreen<Content: View>: View {
    @Environment(RobotStore.self) private var store
    let title: String
    let content: Content

    init(_ title: String, @ViewBuilder content: () -> Content) {
        self.title = title
        self.content = content()
    }

    var body: some View {
        NavigationStack {
            ScrollView {
                VStack(spacing: 14) {
                    if store.connection != .connected {
                        ConnectionBanner()
                    }
                    content
                    Text("APEX QUADRUPED")
                        .font(.system(size: 10, weight: .semibold))
                        .tracking(3)
                        .foregroundStyle(Color(red: 69 / 255, green: 69 / 255, blue: 78 / 255))
                        .padding(.top, 8)
                }
                .padding(.horizontal, 16)
                .padding(.bottom, 24)
                .frame(maxWidth: 640)
                .frame(maxWidth: .infinity)
            }
            .scrollDismissesKeyboard(.interactively)
            .background(ApexBackground())
            .navigationTitle(title)
            .toolbarBackground(Color.apexBG, for: .navigationBar)
            .toolbar {
                ToolbarItem(placement: .topBarTrailing) {
                    ConnectionAccessory()
                }
            }
            .refreshable { await store.refresh() }
        }
    }
}

/// Top right of every tab: the Connect button while the robot is unreachable,
/// otherwise the status light. A separate view so it tracks the store itself.
@MainActor
struct ConnectionAccessory: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        if store.connection == .disconnected {
            ConnectButton()
        } else {
            ConnectionLight()
        }
    }
}

/// Shown in the top right while the robot is unreachable. Opens iOS Wi-Fi settings.
@MainActor
struct ConnectButton: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        Button {
            Haptics.tap()
            store.openWiFiSettings()
        } label: {
            HStack(spacing: 5) {
                Image(systemName: "wifi")
                Text("Connect")
            }
            .font(.system(size: 13, weight: .bold))
            .foregroundStyle(.white)
            .padding(.horizontal, 12)
            .padding(.vertical, 6)
            .background(Color.apexRed, in: Capsule())
        }
        .accessibilityLabel("Open Wi-Fi settings to join the robot's network")
    }
}

@MainActor
struct ConnectionLight: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        HStack(spacing: 6) {
            Circle()
                .fill(color)
                .frame(width: 9, height: 9)
                .shadow(color: color.opacity(0.8), radius: 4)
            Text(label)
                .font(.system(size: 11, weight: .bold, design: .monospaced))
                .foregroundStyle(Color.apexMuted)
        }
        .accessibilityElement(children: .combine)
    }

    private var color: Color {
        switch store.connection {
        case .connected: return .apexGreen
        case .connecting: return .apexAmber
        case .disconnected: return .apexBad
        }
    }

    private var label: String {
        switch store.connection {
        case .connected: return "LIVE"
        case .connecting: return "LINKING"
        case .disconnected: return "OFFLINE"
        }
    }
}

@MainActor
struct ConnectionBanner: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        HStack(alignment: .top, spacing: 10) {
            if store.connection == .connecting {
                ProgressView().tint(.white)
            } else {
                Image(systemName: "wifi.exclamationmark")
                    .font(.system(size: 18, weight: .semibold))
            }
            VStack(alignment: .leading, spacing: 3) {
                Text(store.connection == .connecting ? "Connecting to \(store.host)" : "Robot not reachable")
                    .font(.system(size: 14, weight: .bold))
                Text("Join the APEX Wi-Fi network (APEXWifi). If it still does not connect, check the address in Settings.")
                    .font(.system(size: 12))
                    .foregroundStyle(Color(white: 0.85))
                    .fixedSize(horizontal: false, vertical: true)
                if store.connection == .disconnected {
                    Button {
                        Haptics.tap()
                        store.openWiFiSettings()
                    } label: {
                        Label("Open Wi-Fi settings", systemImage: "wifi")
                            .font(.system(size: 14, weight: .bold))
                            .foregroundStyle(.white)
                            .padding(.horizontal, 14)
                            .padding(.vertical, 8)
                            .background(Color.apexRed, in: Capsule())
                    }
                    .buttonStyle(.plain)
                    .padding(.top, 6)
                }
            }
            Spacer(minLength: 0)
        }
        .foregroundStyle(.white)
        .padding(12)
        .background(Color(red: 40 / 255, green: 12 / 255, blue: 16 / 255), in: RoundedRectangle(cornerRadius: 10))
        .overlay(RoundedRectangle(cornerRadius: 10).strokeBorder(Color.apexRed.opacity(0.6), lineWidth: 1))
    }
}

struct ToastView: View {
    let text: String
    var success = false

    var body: some View {
        HStack(spacing: 8) {
            Image(systemName: success ? "checkmark.circle.fill" : "exclamationmark.circle.fill")
                .foregroundStyle(success ? Color.apexGreen : Color.apexRedHi)
            Text(text)
                .font(.system(size: 14, weight: .semibold))
                .foregroundStyle(.white)
                .fixedSize(horizontal: false, vertical: true)
            Spacer(minLength: 0)
        }
        .padding(12)
        .background(.ultraThinMaterial, in: RoundedRectangle(cornerRadius: 12))
        .overlay(RoundedRectangle(cornerRadius: 12).strokeBorder(Color.apexRed.opacity(0.7), lineWidth: 1))
        .shadow(color: .black.opacity(0.5), radius: 12, y: 4)
    }
}

/// HOMED n/4, STANDING, WALKING pills from the web header.
@MainActor
struct StatusPills: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        let s = store.status
        HStack(spacing: 8) {
            Pill(text: "HOMED \(s.homedCount)/4", live: s.allHomed)
            Pill(text: "STANDING", live: s.isStanding)
            Pill(text: "WALKING", live: s.isWalking)
        }
        .frame(maxWidth: .infinity)
    }
}
