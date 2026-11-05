import SwiftUI

@MainActor
struct SystemsView: View {
    var body: some View {
        ApexScreen("Systems") {
            AvoidanceCard()
            HealthCard()
            CurrentsCard()
            DebugCard()
        }
    }
}

@MainActor
struct AvoidanceCard: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        let s = store.status
        ApexCard("Obstacle Avoidance", info: """
            Camera based. Works in manual and NAV mode: it takes whichever direction the robot wants \
            to travel and returns the nearest one that is actually clear. Fully blocked means stride \
            drops to zero and it marches in place, still standing (unlike STOP). While ON, the video \
            feed shows detection bins (red = blocked).
            """, chip: s.avoidIsAvailable ? (s.avoidIsOn ? "ON" : "OFF") : "NO MODEL", chipLive: s.avoidIsOn) {
            Button {
                store.toggleAvoid()
            } label: {
                Text(!s.avoidIsAvailable ? "Avoidance: no model" : (s.avoidIsOn ? "Avoidance: on" : "Avoidance: off"))
            }
            .apexButton(s.avoidIsOn ? .on : .off, tall: true)
            .disabled(!store.isConnected || !s.avoidIsAvailable)

            Group {
                if !s.avoidIsAvailable {
                    Text("Depth model not loaded. Run python3 vision_obstacle.py --download on the Pi.")
                        .foregroundStyle(Color.apexMuted)
                } else if s.avoidIsOn {
                    let st = AvoidStateInfo.from(s.avoidState)
                    VStack(alignment: .leading, spacing: 4) {
                        (Text(st.label).bold().foregroundStyle(st.color)
                            + Text("  \(st.detail)").foregroundStyle(Color.apexText))
                        Text("steer \(s.avoidSteer >= 0 ? "+" : "")\(s.avoidSteer)°  ·  stride \(s.avoidStride)%")
                            .foregroundStyle(Color(red: 111 / 255, green: 111 / 255, blue: 121 / 255))
                    }
                } else {
                    Text("Off. Steering is unmodified.")
                        .foregroundStyle(Color.apexMuted)
                }
            }
            .font(.system(size: 13, design: .monospaced))
            .fixedSize(horizontal: false, vertical: true)
            .frame(maxWidth: .infinity, alignment: .leading)
            .padding(10)
            .background(Color.apexWell, in: RoundedRectangle(cornerRadius: 8))
            .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(Color(white: 0.13), lineWidth: 1))
        }
    }
}

@MainActor
struct HealthCard: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        let s = store.status
        let down = s.offlineSubsystems.count
        ApexCard("Subsystems", info: "Health bits reported by pi5_main.py. A subsystem can also drop out mid-run, for example if the compass stops reading.",
                 chip: store.isConnected ? (down == 0 ? "ALL UP" : "\(down) DOWN") : "--",
                 chipLive: store.isConnected && down == 0) {
            LazyVGrid(columns: [GridItem(.flexible(), spacing: 8), GridItem(.flexible(), spacing: 8)], spacing: 8) {
                ForEach(Subsystem.all.indices, id: \.self) { i in
                    let up = s.subsystemUp(i)
                    let color: Color = !store.isConnected ? .apexMuted : (up ? .apexGreen : .apexAmber)
                    HStack(spacing: 8) {
                        Image(systemName: Subsystem.icons[i])
                            .font(.system(size: 13))
                            .foregroundStyle(color)
                            .frame(width: 18)
                        Text(Subsystem.all[i])
                            .font(.system(size: 13, weight: .semibold))
                            .foregroundStyle(Color.apexText)
                            .lineLimit(1)
                            .minimumScaleFactor(0.8)
                        Spacer(minLength: 0)
                        Circle().fill(color).frame(width: 8, height: 8)
                            .shadow(color: color.opacity(0.8), radius: 3)
                    }
                    .padding(.horizontal, 10)
                    .frame(height: 40)
                    .background(Color.apexWell, in: RoundedRectangle(cornerRadius: 8))
                    .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(Color.apexLine, lineWidth: 1))
                    .accessibilityElement(children: .combine)
                    .accessibilityValue(up ? "online" : "offline")
                }
            }
        }
    }
}

@MainActor
struct CurrentsCard: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        let s = store.status
        ApexCard("Motor Currents", info: """
            Live current per motor from each BTS7960's current-sense outputs, shown as right side / \
            left side of the driver (RPWM / LPWM). Only the side that is driving reports, so one value \
            is normally near zero. Updates about once a second. -- means that leg is not reporting or \
            its ADC reads as dead.
            """, chip: "\(s.legsReportingCurrent)/4 LEGS", chipLive: s.legsReportingCurrent == 4) {
            Grid(alignment: .trailing, horizontalSpacing: 6, verticalSpacing: 0) {
                GridRow {
                    Text("Leg").gridColumnAlignment(.leading)
                    ForEach(Joint.names, id: \.self) { Text("\($0) R/L") }
                }
                .font(.system(size: 11, weight: .semibold, design: .monospaced))
                .foregroundStyle(Color.apexMuted)
                .padding(.bottom, 6)

                ForEach(0..<4, id: \.self) { leg in
                    Divider().overlay(Color.apexLine)
                    GridRow {
                        Text(Leg.names[leg])
                            .foregroundStyle(Color.apexText)
                            .gridColumnAlignment(.leading)
                        ForEach(0..<3, id: \.self) { joint in
                            cell(s, leg: leg, joint: joint)
                        }
                    }
                    .font(.system(size: 12, design: .monospaced))
                    .padding(.vertical, 8)
                }
            }
            .frame(maxWidth: .infinity)
        }
    }

    @ViewBuilder
    private func cell(_ s: ApexStatus, leg: Int, joint: Int) -> some View {
        if let r = s.current(leg: leg, joint: joint, side: 0), let l = s.current(leg: leg, joint: joint, side: 1) {
            Text(amps(r) + "/" + amps(l))
                .foregroundStyle(color(forMilliamps: max(r, l)))
                .lineLimit(1)
                .minimumScaleFactor(0.7)
        } else {
            Text("--").foregroundStyle(Color.apexMuted)
        }
    }

    // Display-only highlight thresholds. Motor stall is about 9.1 A, so 8 A reads
    // as near stall. Tune these to taste; nothing on the robot uses them.
    private static let warnMilliamps = 5000
    private static let highMilliamps = 8000

    private func color(forMilliamps mA: Int) -> Color {
        if mA >= Self.highMilliamps { return .apexBad }
        if mA >= Self.warnMilliamps { return .apexAmber }
        return .apexText
    }

    private func amps(_ mA: Int) -> String {
        String(format: "%.2f", Double(mA) / 1000)
    }
}

@MainActor
struct DebugCard: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        let s = store.status
        ApexCard("Debug: Leg Deactivation", info: """
            Testing only. A deactivated leg is still fully computed every gait step, it just receives \
            no wire signal, so it holds its last position while the other three keep walking.
            """) {
            LazyVGrid(columns: [GridItem(.flexible(), spacing: 8), GridItem(.flexible(), spacing: 8)], spacing: 8) {
                ForEach(0..<4, id: \.self) { leg in
                    let off = s.isDeactivated(leg)
                    Button {
                        store.toggleLeg(leg)
                    } label: {
                        Text((off ? "Reactivate " : "Deactivate ") + Leg.names[leg])
                    }
                    .apexButton(off ? .degraded : .normal)
                    .disabled(!store.isConnected)
                }
            }
        }
    }
}
