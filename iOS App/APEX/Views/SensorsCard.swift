import SwiftUI

/// Systems tab: every value from GET /api/state in labeled groups.
@MainActor
struct SensorsCard: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        let st = store.state
        ApexCard("Sensor Readout", info: """
            Live values from the robot's /api/state endpoint. The power reading comes from an INA226 \
            that sits on the Pi's 5 V supply rail only. It is not the battery and not the 12 V motor \
            supply. Current is the shunt voltage divided by a 0.002 ohm shunt. "--" means the robot \
            reported no value.
            """, chip: chip, chipLive: st != nil && store.stateMissing == false) {
            if store.stateMissing {
                Text("This robot does not have /api/state yet. Apply the stream_server.py patch on the Pi and restart the container.")
                    .font(.system(size: 13))
                    .foregroundStyle(Color.apexAmber)
                    .fixedSize(horizontal: false, vertical: true)
            } else if let st {
                VStack(alignment: .leading, spacing: 14) {
                    powerGroup(st.power)
                    gpsGroup(st.gps)
                    subsystemGroup(st.health, camera: st.camera)
                    robotGroup(st.robot, meta: st.meta)
                    navGroup(st.navigation, avoid: st.avoidance)
                    legGroup(st.legs)
                }
            } else {
                Text(store.isConnected ? "Waiting for data..." : "Not connected.")
                    .font(.system(size: 13))
                    .foregroundStyle(Color.apexMuted)
            }
        }
    }

    private var chip: String {
        if store.stateMissing { return "NO ENDPOINT" }
        return store.state == nil ? "--" : "LIVE"
    }

    // MARK: Groups

    private func powerGroup(_ p: ApexState.Power?) -> some View {
        let valid = p?.valid == true
        return group("Pi 5 V rail (INA226)") {
            HStack(spacing: 8) {
                tile("Current", valid ? p?.rail_5v_amps : nil, "A", digits: 2)
                tile("Voltage", valid ? p?.rail_5v_volts : nil, "V", digits: 2)
                tile("Power", valid ? p?.rail_5v_watts : nil, "W", digits: 1)
            }
            if !valid {
                note("INA226 not reporting.")
            } else if let v = p?.rail_5v_volts, v < 4.75 {
                note("Below 4.75 V, the robot's low-voltage alert level.", warn: true)
            }
        }
    }

    private func gpsGroup(_ g: ApexState.GPS?) -> some View {
        group("GPS and compass") {
            row("Fix", yesNo(g?.fix), good: g?.fix)
            row("Satellites", g?.satellites.map(String.init) ?? "--")
            row("Latitude", fmt(g?.lat, 6, "°"))
            row("Longitude", fmt(g?.lon, 6, "°"))
            row("Heading", headingText(g))
        }
    }

    private func subsystemGroup(_ h: ApexState.Health?, camera: ApexState.Camera?) -> some View {
        let s = h?.subsystems
        return group("Subsystems") {
            row("Overall", h?.ok.map { $0 ? "All up" : "Down: " + (h?.offline ?? []).joined(separator: ", ") } ?? "--", good: h?.ok)
            row("IMU", upDown(s?.imu), good: s?.imu)
            row("GPS", upDown(s?.gps), good: s?.gps)
            row("Compass", upDown(s?.compass), good: s?.compass)
            row("Power monitor", upDown(s?.power), good: s?.power)
            row("Camera", upDown(s?.camera), good: s?.camera)
            row("Audio", upDown(s?.audio), good: s?.audio)
            row("Camera frame age", fmt(camera?.frame_age_s, 2, " s"), good: camera?.live)
        }
    }

    private func robotGroup(_ r: ApexState.Robot?, meta: ApexState.Meta?) -> some View {
        group("Robot") {
            row("All legs homed", yesNo(r?.all_homed), good: r?.all_homed)
            row("Standing", yesNo(r?.standing))
            row("Walking", yesNo(r?.walking))
            row("Steering", fmt(r?.steering_deg, 0, "°"))
            row("Uptime", uptime(meta?.uptime_s))
            row("Schema", meta?.schema.map(String.init) ?? "--")
        }
    }

    private func navGroup(_ n: ApexState.Navigation?, avoid a: ApexState.Avoidance?) -> some View {
        group("Navigation and avoidance") {
            row("Autonomous", yesNo(n?.autonomous))
            row("Paused", yesNo(n?.paused))
            row("Waypoint", waypointText(n))
            row("Route complete", yesNo(n?.route_complete))
            row("Avoidance model", yesNo(a?.available))
            row("Avoidance on", yesNo(a?.enabled))
            row("Avoidance state", a?.state?.uppercased() ?? "--")
            row("Avoid steer", fmt(a?.steer_deg, 0, "°"))
            row("Stride", fmt(a?.stride_pct, 0, "%"))
        }
    }

    private func legGroup(_ legs: [ApexState.StateLeg]?) -> some View {
        group("Legs (motor current, mA, right/left)") {
            if let legs, !legs.isEmpty {
                ForEach(legs.indices, id: \.self) { i in
                    let l = legs[i]
                    VStack(alignment: .leading, spacing: 3) {
                        HStack {
                            Text(l.name ?? "Leg \(i)")
                                .font(.system(size: 13, weight: .bold, design: .monospaced))
                                .foregroundStyle(Color.apexText)
                            Spacer()
                            Text(legFlags(l))
                                .font(.system(size: 11, design: .monospaced))
                                .foregroundStyle(l.deactivated == true ? Color.apexAmber : Color.apexMuted)
                        }
                        currentLine("roll", l.currents_ma?.roll)
                        currentLine("pitch", l.currents_ma?.pitch)
                        currentLine("knee", l.currents_ma?.knee)
                    }
                    if i < legs.count - 1 { Divider().overlay(Color.apexLine) }
                }
            } else {
                note("No leg data.")
            }
        }
    }

    // MARK: Pieces

    private func group<C: View>(_ title: String, @ViewBuilder _ content: () -> C) -> some View {
        VStack(alignment: .leading, spacing: 6) {
            Text(title.uppercased())
                .font(.system(size: 11, weight: .bold))
                .tracking(1)
                .foregroundStyle(Color.apexMuted)
            VStack(alignment: .leading, spacing: 6) { content() }
                .padding(10)
                .frame(maxWidth: .infinity, alignment: .leading)
                .background(Color.apexWell, in: RoundedRectangle(cornerRadius: 8))
                .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(Color.apexLine, lineWidth: 1))
        }
    }

    private func tile(_ label: String, _ value: Double?, _ unit: String, digits: Int) -> some View {
        VStack(spacing: 2) {
            Text(value.map { String(format: "%.\(digits)f", $0) } ?? "--")
                .font(.system(size: 20, weight: .bold, design: .monospaced))
                .foregroundStyle(value == nil ? Color.apexMuted : Color.apexText)
                .minimumScaleFactor(0.7)
                .lineLimit(1)
            Text("\(label) (\(unit))")
                .font(.system(size: 10, weight: .semibold))
                .foregroundStyle(Color.apexMuted)
        }
        .frame(maxWidth: .infinity)
    }

    private func row(_ label: String, _ value: String, good: Bool? = nil) -> some View {
        HStack(alignment: .firstTextBaseline) {
            Text(label)
                .foregroundStyle(Color.apexMuted)
            Spacer(minLength: 12)
            Text(value)
                .foregroundStyle(color(good))
                .multilineTextAlignment(.trailing)
        }
        .font(.system(size: 13, design: .monospaced))
    }

    private func note(_ text: String, warn: Bool = false) -> some View {
        Text(text)
            .font(.system(size: 12))
            .foregroundStyle(warn ? Color.apexAmber : Color.apexMuted)
            .fixedSize(horizontal: false, vertical: true)
    }

    private func currentLine(_ name: String, _ s: ApexState.Side?) -> some View {
        HStack {
            Text(name).foregroundStyle(Color.apexMuted)
            Spacer()
            Text("\(num(s?.right)) / \(num(s?.left))").foregroundStyle(Color.apexText)
        }
        .font(.system(size: 12, design: .monospaced))
    }

    // MARK: Formatting

    private func color(_ good: Bool?) -> Color {
        guard let good else { return .apexText }
        return good ? .apexGreen : .apexAmber
    }

    private func yesNo(_ b: Bool?) -> String { b.map { $0 ? "Yes" : "No" } ?? "--" }
    private func upDown(_ b: Bool?) -> String { b.map { $0 ? "Online" : "Offline" } ?? "--" }
    private func num(_ v: Double?) -> String { v.map { String(Int($0.rounded())) } ?? "--" }

    private func fmt(_ v: Double?, _ digits: Int, _ unit: String) -> String {
        v.map { String(format: "%.\(digits)f", $0) + unit } ?? "--"
    }

    private func headingText(_ g: ApexState.GPS?) -> String {
        guard let h = g?.heading_deg else { return "--" }
        let pt = g?.compass_point.map { " " + $0 } ?? ""
        return String(format: "%.0f°", h) + pt + " (true)"
    }

    private func waypointText(_ n: ApexState.Navigation?) -> String {
        guard let t = n?.waypoint_total else { return "--" }
        guard let i = n?.waypoint_index else { return "-- / \(t)" }
        return "index \(i) / \(t) total"
    }

    private func legFlags(_ l: ApexState.StateLeg) -> String {
        var parts: [String] = []
        if let h = l.homed { parts.append(h ? "homed" : "not homed") }
        if l.deactivated == true { parts.append("deactivated") }
        return parts.joined(separator: " · ")
    }

    private func uptime(_ s: Double?) -> String {
        guard let s else { return "--" }
        let t = Int(s)
        let h = t / 3600, m = (t % 3600) / 60, sec = t % 60
        return h > 0 ? "\(h)h \(m)m \(sec)s" : "\(m)m \(sec)s"
    }
}
