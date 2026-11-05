import SwiftUI

/// The live camera image plus a small state overlay. Registers itself as a
/// consumer of the shared stream so it only downloads while on screen.
@MainActor
struct FeedView: View {
    @Environment(RobotStore.self) private var store
    let cornerRadius: CGFloat
    let showsBorder: Bool

    init(cornerRadius: CGFloat = 10, showsBorder: Bool = true) {
        self.cornerRadius = cornerRadius
        self.showsBorder = showsBorder
    }

    var body: some View {
        let feed = store.feed
        ZStack {
            Color.black
            if let img = feed.image {
                Image(uiImage: img)
                    .resizable()
                    .interpolation(.medium)
                    .scaledToFit()
            }
            if feed.state != .streaming {
                VStack(spacing: 8) {
                    if feed.state == .connecting || feed.state == .idle {
                        ProgressView().tint(.white)
                    } else {
                        Image(systemName: "video.slash.fill")
                            .font(.system(size: 24))
                            .foregroundStyle(Color.apexMuted)
                    }
                    Text(stateText(feed.state))
                        .font(.system(size: 12, weight: .semibold, design: .monospaced))
                        .foregroundStyle(Color.apexMuted)
                }
                .padding(10)
                .background(.black.opacity(feed.image == nil ? 0 : 0.55), in: RoundedRectangle(cornerRadius: 8))
            }
        }
        .overlay(alignment: .topLeading) {
            if feed.state == .streaming {
                HStack(spacing: 5) {
                    Circle().fill(Color.apexRedHi).frame(width: 7, height: 7)
                    Text("LIVE  \(feed.fps) FPS")
                        .font(.system(size: 10, weight: .bold, design: .monospaced))
                }
                .foregroundStyle(.white)
                .padding(.horizontal, 7)
                .padding(.vertical, 4)
                .background(.black.opacity(0.55), in: Capsule())
                .padding(8)
            }
        }
        .clipShape(RoundedRectangle(cornerRadius: cornerRadius, style: .continuous))
        .overlay {
            if showsBorder {
                RoundedRectangle(cornerRadius: cornerRadius, style: .continuous)
                    .strokeBorder(Color.apexLine, lineWidth: 1)
            }
        }
        .onAppear { store.feed.acquire() }
        .onDisappear { store.feed.release() }
    }

    private func stateText(_ s: MJPEGStream.State) -> String {
        switch s {
        case .idle, .connecting: return "Connecting to camera"
        case .failed: return "No video. Retrying"
        case .streaming: return ""
        }
    }
}

/// Full-screen feed with driving controls laid over it. Works in portrait and landscape.
@MainActor
struct FullscreenFeedView: View {
    @Environment(RobotStore.self) private var store
    @Environment(\.dismiss) private var dismiss

    var body: some View {
        ZStack {
            Color.black.ignoresSafeArea()
            FeedView(cornerRadius: 0, showsBorder: false)
                .ignoresSafeArea()

            VStack {
                HStack {
                    StatusPills()
                    Button {
                        dismiss()
                    } label: {
                        Image(systemName: "xmark")
                            .font(.system(size: 16, weight: .bold))
                            .foregroundStyle(.white)
                            .frame(width: 40, height: 40)
                            .background(.black.opacity(0.6), in: Circle())
                    }
                    .accessibilityLabel("Close full screen")
                }
                Spacer()
                VStack(spacing: 10) {
                    SteeringControl(compact: true)
                    HStack(spacing: 8) {
                        Button("Stand") { store.stand() }
                            .apexButton()
                            .disabled(!store.status.allHomed || !store.isConnected)
                        Button(store.status.isWalking ? "Walking" : "Go") { store.go() }
                            .apexButton(.primary)
                            .disabled(!store.isConnected || !(store.status.allHomed && store.status.isStanding)
                                      || store.status.isWalking)
                        Button("Stop") { store.stopMotors() }
                            .apexButton(.stop)
                            .disabled(!store.isConnected)
                    }
                }
                .padding(12)
                .background(.black.opacity(0.55), in: RoundedRectangle(cornerRadius: 14))
                .frame(maxWidth: 560)
            }
            .padding(12)
        }
        .statusBarHidden()
    }
}
