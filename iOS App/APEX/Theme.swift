import SwiftUI
import UIKit

// APEX red/black theme, same tokens as dashboard_page.py.
// Colour semantics: red = engaged/live, dark grey = idle, amber = degraded.

extension Color {
    static let apexBG = Color(red: 8 / 255, green: 8 / 255, blue: 10 / 255)
    static let apexPanel = Color(red: 20 / 255, green: 20 / 255, blue: 23 / 255)
    static let apexPanel2 = Color(red: 15 / 255, green: 15 / 255, blue: 18 / 255)
    static let apexWell = Color(red: 12 / 255, green: 12 / 255, blue: 15 / 255)
    static let apexLine = Color(red: 42 / 255, green: 42 / 255, blue: 49 / 255)
    static let apexRed = Color(red: 224 / 255, green: 16 / 255, blue: 32 / 255)
    static let apexRedHi = Color(red: 1, green: 42 / 255, blue: 58 / 255)
    static let apexText = Color(red: 233 / 255, green: 233 / 255, blue: 236 / 255)
    static let apexMuted = Color(red: 139 / 255, green: 139 / 255, blue: 149 / 255)
    static let apexAmber = Color(red: 240 / 255, green: 160 / 255, blue: 32 / 255)
    static let apexGreen = Color(red: 74 / 255, green: 222 / 255, blue: 128 / 255)
    static let apexBad = Color(red: 1, green: 59 / 255, blue: 48 / 255)
}

extension LinearGradient {
    static let apexRed = LinearGradient(colors: [.apexRedHi, .apexRed], startPoint: .top, endPoint: .bottom)
}

// MARK: Buttons

enum ApexButtonKind {
    case normal, primary, stop, on, off, degraded, go, hold
}

struct ApexButtonStyle: ButtonStyle {
    let kind: ApexButtonKind
    let tall: Bool
    @Environment(\.isEnabled) private var isEnabled

    init(kind: ApexButtonKind = .normal, tall: Bool = false) {
        self.kind = kind
        self.tall = tall
    }

    private struct Palette {
        let bg: AnyShapeStyle
        let border: Color
        let fg: Color
        let glow: Color
    }

    private var palette: Palette {
        switch kind {
        case .normal:
            return Palette(bg: AnyShapeStyle(Color(red: 23 / 255, green: 23 / 255, blue: 27 / 255)),
                           border: Color(red: 46 / 255, green: 46 / 255, blue: 54 / 255), fg: .apexText, glow: .clear)
        case .primary, .on:
            return Palette(bg: AnyShapeStyle(LinearGradient.apexRed),
                           border: Color(red: 1, green: 75 / 255, blue: 87 / 255), fg: .white,
                           glow: Color.apexRed.opacity(0.45))
        case .stop:
            return Palette(bg: AnyShapeStyle(Color(red: 36 / 255, green: 10 / 255, blue: 13 / 255)),
                           border: Color(red: 122 / 255, green: 16 / 255, blue: 24 / 255),
                           fg: Color(red: 1, green: 144 / 255, blue: 152 / 255), glow: .clear)
        case .off:
            return Palette(bg: AnyShapeStyle(Color(red: 23 / 255, green: 23 / 255, blue: 27 / 255)),
                           border: Color(red: 46 / 255, green: 46 / 255, blue: 54 / 255), fg: .apexMuted, glow: .clear)
        case .degraded:
            return Palette(bg: AnyShapeStyle(Color(red: 42 / 255, green: 28 / 255, blue: 6 / 255)),
                           border: .apexAmber, fg: Color(red: 1, green: 206 / 255, blue: 122 / 255), glow: .clear)
        case .go:
            return Palette(bg: AnyShapeStyle(Color(red: 12 / 255, green: 42 / 255, blue: 22 / 255)),
                           border: Color(red: 28 / 255, green: 122 / 255, blue: 62 / 255),
                           fg: Color(red: 127 / 255, green: 240 / 255, blue: 168 / 255), glow: .clear)
        case .hold:
            return Palette(bg: AnyShapeStyle(Color(red: 42 / 255, green: 36 / 255, blue: 6 / 255)),
                           border: .apexAmber, fg: Color(red: 1, green: 206 / 255, blue: 122 / 255), glow: .clear)
        }
    }

    func makeBody(configuration: Configuration) -> some View {
        let p = palette
        return configuration.label
            .font(.system(size: tall ? 15 : 13, weight: .bold))
            .tracking(1.1)
            .textCase(.uppercase)
            .lineLimit(1)
            .minimumScaleFactor(0.6)
            .foregroundStyle(p.fg)
            .padding(.horizontal, 8)
            .frame(maxWidth: .infinity, minHeight: tall ? 54 : 46)
            .background(RoundedRectangle(cornerRadius: 10, style: .continuous).fill(p.bg))
            .overlay(RoundedRectangle(cornerRadius: 10, style: .continuous).strokeBorder(p.border, lineWidth: 1))
            .shadow(color: isEnabled ? p.glow : .clear, radius: 9)
            .opacity(isEnabled ? (configuration.isPressed ? 0.75 : 1) : 0.38)
            .scaleEffect(configuration.isPressed ? 0.97 : 1)
            .animation(.easeOut(duration: 0.12), value: configuration.isPressed)
            .contentShape(Rectangle())
    }
}

extension View {
    func apexButton(_ kind: ApexButtonKind = .normal, tall: Bool = false) -> some View {
        buttonStyle(ApexButtonStyle(kind: kind, tall: tall))
    }
}

// MARK: Cards

struct ApexCard<Content: View>: View {
    let title: String
    let info: String?
    let chip: String?
    let chipLive: Bool
    let content: Content
    @State private var showInfo = false

    init(_ title: String, info: String? = nil, chip: String? = nil, chipLive: Bool = false,
         @ViewBuilder content: () -> Content) {
        self.title = title
        self.info = info
        self.chip = chip
        self.chipLive = chipLive
        self.content = content()
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack(spacing: 8) {
                Text(title.uppercased())
                    .font(.system(size: 12, weight: .heavy))
                    .tracking(2.4)
                    .foregroundStyle(Color(white: 0.85))
                if info != nil {
                    Button {
                        withAnimation(.easeOut(duration: 0.15)) { showInfo.toggle() }
                    } label: {
                        Image(systemName: showInfo ? "info.circle.fill" : "info.circle")
                            .font(.system(size: 15))
                            .foregroundStyle(Color.apexRedHi)
                    }
                    .buttonStyle(.plain)
                    .accessibilityLabel("About \(title)")
                }
                Spacer(minLength: 0)
                if let chip {
                    Pill(text: chip, live: chipLive)
                }
            }
            if showInfo, let info {
                Text(info)
                    .font(.system(size: 13))
                    .foregroundStyle(Color(red: 160 / 255, green: 160 / 255, blue: 170 / 255))
                    .fixedSize(horizontal: false, vertical: true)
                    .padding(10)
                    .frame(maxWidth: .infinity, alignment: .leading)
                    .background(Color.apexWell, in: RoundedRectangle(cornerRadius: 8))
                    .overlay(alignment: .leading) {
                        Rectangle().fill(Color.apexRed).frame(width: 3)
                            .clipShape(RoundedRectangle(cornerRadius: 1.5))
                    }
                    .transition(.opacity)
            }
            content
        }
        .padding(14)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(
            LinearGradient(colors: [.apexPanel, .apexPanel2], startPoint: .top, endPoint: .bottom),
            in: RoundedRectangle(cornerRadius: 14, style: .continuous))
        .overlay(RoundedRectangle(cornerRadius: 14, style: .continuous).strokeBorder(Color.apexLine, lineWidth: 1))
        .overlay(alignment: .top) {
            LinearGradient(colors: [.clear, .apexRed, .clear], startPoint: .leading, endPoint: .trailing)
                .frame(height: 2)
                .padding(.horizontal, 14)
        }
    }
}

struct Pill: View {
    let text: String
    var live = false
    var tint: Color? = nil

    var body: some View {
        Text(text)
            .font(.system(size: 11, weight: .bold, design: .monospaced))
            .tracking(1.2)
            .lineLimit(1)
            .padding(.horizontal, 10)
            .padding(.vertical, 5)
            .foregroundStyle(live ? Color.white : (tint ?? Color.apexMuted))
            .background {
                Capsule().fill(live ? AnyShapeStyle(LinearGradient.apexRed)
                                    : AnyShapeStyle(Color(red: 19 / 255, green: 19 / 255, blue: 22 / 255)))
            }
            .overlay(Capsule().strokeBorder(live ? Color(red: 1, green: 75 / 255, blue: 87 / 255)
                                                 : (tint ?? Color.apexLine), lineWidth: 1))
            .shadow(color: live ? Color.apexRed.opacity(0.45) : .clear, radius: 7)
    }
}

/// Striped warning banner (red = blocking, amber = degraded), like the web page.
struct Banner: View {
    let text: String
    var amber = false

    var body: some View {
        HStack(alignment: .top, spacing: 8) {
            Image(systemName: "exclamationmark.triangle.fill")
            Text(text).fixedSize(horizontal: false, vertical: true)
            Spacer(minLength: 0)
        }
        .font(.system(size: 13, weight: .semibold))
        .foregroundStyle(.white)
        .padding(12)
        .background(amber ? Color(red: 122 / 255, green: 92 / 255, blue: 16 / 255)
                          : Color(red: 157 / 255, green: 12 / 255, blue: 23 / 255),
                    in: RoundedRectangle(cornerRadius: 10))
        .overlay(RoundedRectangle(cornerRadius: 10)
            .strokeBorder(amber ? Color.apexAmber : Color(red: 1, green: 75 / 255, blue: 87 / 255), lineWidth: 1))
    }
}

/// Dark background with the red glow at the top, used behind every screen.
struct ApexBackground: View {
    var body: some View {
        ZStack {
            Color.apexBG
            RadialGradient(colors: [Color.apexRed.opacity(0.18), .clear],
                           center: .init(x: 0.5, y: -0.05), startRadius: 0, endRadius: 520)
        }
        .ignoresSafeArea()
    }
}

// MARK: Haptics

@MainActor
enum Haptics {
    static func tap() { UIImpactFeedbackGenerator(style: .medium).impactOccurred() }
    static func light() { UIImpactFeedbackGenerator(style: .light).impactOccurred() }
    static func success() { UINotificationFeedbackGenerator().notificationOccurred(.success) }
    static func error() { UINotificationFeedbackGenerator().notificationOccurred(.error) }
}
