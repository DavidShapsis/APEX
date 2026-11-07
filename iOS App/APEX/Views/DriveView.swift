import SwiftUI

@MainActor
struct DriveView: View {
    @Environment(RobotStore.self) private var store
    @State private var fullscreen = false

    var body: some View {
        let s = store.status
        ApexScreen("Drive") {
            StatusPills()

            if store.isConnected && !s.allHomed {
                Banner(text: "Not every leg is homed. STAND and GO stay disabled until all four are homed.")
            }
            if store.isConnected && !s.offlineSubsystems.isEmpty {
                Banner(text: degradedText(s.offlineSubsystems), amber: true)
            }

            ApexCard("Live Feed") {
                FeedView()
                    .aspectRatio(4 / 3, contentMode: .fit)
                    .overlay(alignment: .bottomTrailing) {
                        Button {
                            fullscreen = true
                        } label: {
                            Image(systemName: "arrow.up.left.and.arrow.down.right")
                                .font(.system(size: 14, weight: .bold))
                                .foregroundStyle(.white)
                                .frame(width: 34, height: 34)
                                .background(.black.opacity(0.6), in: Circle())
                        }
                        .padding(8)
                        .accessibilityLabel("Full screen video")
                    }
                CaptureButtons()
            }

            ApexCard("Steering", info: """
                Turn command. LEFT and RIGHT are 45° presets: a moderate arc that keeps about half \
                the forward speed. Use the slider for anything else. A small angle keeps most of the \
                forward speed, and the ±90° ends are a spin in place. The robot only responds once \
                it is walking (Home all four legs, then STAND, then GO). NAV MODE hands steering to \
                autonomous GPS waypoint following.
                """) {
                SteeringControl()
                if s.navOn {
                    Text("NAV MODE is on. GPS waypoint following is steering.")
                        .font(.system(size: 12))
                        .foregroundStyle(Color.apexAmber)
                } else if store.isConnected && !s.isWalking {
                    Text("Steering takes effect once the robot is walking.")
                        .font(.system(size: 12))
                        .foregroundStyle(Color.apexMuted)
                }
            }

            StartupCard()
        }
        .fullScreenCover(isPresented: $fullscreen) {
            FullscreenFeedView()
                .environment(store)
        }
    }

    private func degradedText(_ down: [String]) -> String {
        "Running degraded: \(down.joined(separator: ", ")) \(down.count == 1 ? "is" : "are") offline. "
            + "Check the boot screen or the Pi console."
    }
}

/// Direction readout, -90...+90 slider and the LEFT / FWD / RIGHT presets.
@MainActor
struct SteeringControl: View {
    @Environment(RobotStore.self) private var store
    let compact: Bool

    init(compact: Bool = false) {
        self.compact = compact
    }

    var body: some View {
        VStack(spacing: compact ? 6 : 10) {
            if !compact {
                VStack(spacing: 4) {
                    Text("DIRECTION")
                        .font(.system(size: 10, weight: .semibold))
                        .tracking(3)
                        .foregroundStyle(Color.apexMuted)
                    Text("\(store.direction)°")
                        .font(.system(size: 34, weight: .bold, design: .monospaced))
                        .foregroundStyle(.white)
                        .shadow(color: Color.apexRed.opacity(0.55), radius: 10)
                        .contentTransition(.numericText(value: Double(store.direction)))
                }
                .frame(maxWidth: .infinity)
            }

            HStack(spacing: 10) {
                if compact {
                    Text("\(store.direction)°")
                        .font(.system(size: 16, weight: .bold, design: .monospaced))
                        .foregroundStyle(.white)
                        .frame(width: 52, alignment: .leading)
                }
                Text("-90").font(.system(size: 10, design: .monospaced)).foregroundStyle(Color.apexMuted)
                DirectionSlider(angle: store.direction) { store.setDirection($0) }
                Text("+90").font(.system(size: 10, design: .monospaced)).foregroundStyle(Color.apexMuted)
            }

            HStack(spacing: 8) {
                Button { preset(-45) } label: { Label("Left 45°", systemImage: "arrowtriangle.left.fill") }
                    .apexButton(store.direction == -45 ? .on : .normal)
                Button { preset(0) } label: { Label("Fwd", systemImage: "arrowtriangle.up.fill") }
                    .apexButton(store.direction == 0 ? .on : .normal)
                Button { preset(45) } label: { Label("Right 45°", systemImage: "arrowtriangle.right.fill") }
                    .apexButton(store.direction == 45 ? .on : .normal)
            }
            .labelStyle(.titleOnly)
        }
        .disabled(!store.isConnected)
    }

    private func preset(_ a: Int) {
        Haptics.tap()
        store.setDirection(a)
    }
}

/// Direction slider, -90...+90: a solid red track from end to end with a white thumb.
/// Custom, because the system Slider always colors the part of the track left of the thumb.
@MainActor
struct DirectionSlider: View {
    let angle: Int
    let onChange: (Int) -> Void
    @Environment(\.isEnabled) private var isEnabled

    private let thumb: CGFloat = 30
    private let trackHeight: CGFloat = 10

    init(angle: Int, onChange: @escaping (Int) -> Void) {
        self.angle = angle
        self.onChange = onChange
    }

    var body: some View {
        GeometryReader { geo in
            let usable = max(1, geo.size.width - thumb)
            let thumbX = usable * CGFloat(angle + 90) / 180
            ZStack(alignment: .leading) {
                Capsule()
                    .fill(Color.apexRed)
                    .frame(height: trackHeight)
                    .shadow(color: Color.apexRed.opacity(0.5), radius: 5)
                Circle()
                    .fill(Color.white)
                    .frame(width: thumb, height: thumb)
                    .shadow(color: .black.opacity(0.45), radius: 3, y: 1)
                    .offset(x: thumbX)
            }
            .frame(maxHeight: .infinity)
            .contentShape(Rectangle())
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { drag in
                        guard isEnabled else { return }
                        let t = min(max((drag.location.x - thumb / 2) / usable, 0), 1)
                        commit(Int((t * 180 - 90).rounded()))
                    })
        }
        .frame(height: 44)
        .opacity(isEnabled ? 1 : 0.4)
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("Direction")
        .accessibilityValue("\(angle) degrees")
        .accessibilityAdjustableAction { direction in
            switch direction {
            case .increment: commit(angle + 5)
            case .decrement: commit(angle - 5)
            @unknown default: break
            }
        }
    }

    private func commit(_ value: Int) {
        let a = max(-90, min(90, value))
        guard a != angle else { return }
        if a == 0 || a % 15 == 0 { Haptics.light() }
        onChange(a)
    }
}

/// Home x4, STAND, GO, STOP with the same enable rules as the web page.
@MainActor
struct StartupCard: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        let s = store.status
        let live = store.isConnected
        ApexCard("Startup", info: """
            Position each leg by hand (hip roll 90°, knee locked 180°, hip pitch 0°), then press its \
            Home button. Then STAND, then GO.

            STOP cuts motor power but keeps encoder tracking, so re-homing is not needed to resume. \
            It does NOT lower the robot first. Only use it when the robot is off the ground or supported.

            Pressing STAND while WALKING stops the walk and settles into the stand pose with the legs \
            still powered. Use that, not STOP, to halt a robot that is on its feet.
            """) {
            LazyVGrid(columns: [GridItem(.flexible(), spacing: 8), GridItem(.flexible(), spacing: 8)], spacing: 8) {
                ForEach(0..<4, id: \.self) { leg in
                    Button {
                        store.homeLeg(leg)
                    } label: {
                        Text((s.isHomed(leg) ? "✓ " : "") + "Home " + Leg.names[leg])
                    }
                    .apexButton(s.isHomed(leg) ? .on : .normal)
                    .disabled(!live)
                }
            }
            HStack(spacing: 8) {
                Button("Stand") { store.stand() }
                    .apexButton(s.isStanding && !s.isWalking ? .on : .normal, tall: true)
                    .disabled(!live || !s.allHomed)
                Button(s.isWalking ? "Walking" : "Go") { store.go() }
                    .apexButton(.primary, tall: true)
                    .disabled(!live || !(s.allHomed && s.isStanding) || s.isWalking)
                Button("Stop") { store.stopMotors() }
                    .apexButton(.stop, tall: true)
                    .disabled(!live)
            }
        }
    }
}
