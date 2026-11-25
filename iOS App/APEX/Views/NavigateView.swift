import SwiftUI
import MapKit

struct NavigateView: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        ApexScreen("Navigate") {
            GPSReadoutCard()
            RouteMapCard()
            NavModeCard()
            WaypointEditorCard()
        }
    }
}

// MARK: GPS readout

@MainActor
struct GPSReadoutCard: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        let g = store.status.gps
        ApexCard("Robot GPS", chip: g.hasFix ? "FIX" : "NO FIX", chipLive: g.hasFix) {
            VStack(spacing: 8) {
                row("Position", g.hasFix ? String(format: "%.6f, %.6f", g.lat ?? 0, g.lon ?? 0) : "no fix", ok: g.hasFix)
                row("Heading", headingText(g), ok: g.heading != nil)
                row("Satellites", "\(g.sats)", ok: g.sats > 0)
            }
            .padding(10)
            .background(Color(red: 19 / 255, green: 19 / 255, blue: 22 / 255), in: RoundedRectangle(cornerRadius: 8))
            .overlay(RoundedRectangle(cornerRadius: 8).strokeBorder(Color.apexLine, lineWidth: 1))
        }
    }

    private func headingText(_ g: GPSInfo) -> String {
        guard let h = g.heading else { return "no compass" }
        return String(format: "%.0f° ", h) + (g.compassPoint ?? "")
    }

    private func row(_ label: String, _ value: String, ok: Bool) -> some View {
        HStack {
            Text(label.uppercased())
                .tracking(0.8)
                .foregroundStyle(Color.apexMuted)
            Spacer()
            Text(value)
                .foregroundStyle(ok ? Color(red: 242 / 255, green: 242 / 255, blue: 245 / 255) : Color.apexMuted)
                .textSelection(.enabled)
        }
        .font(.system(size: 13, design: .monospaced))
    }
}

// MARK: Map

@MainActor
struct RouteMapCard: View {
    @Environment(RobotStore.self) private var store
    @State private var camera: MapCameraPosition = .automatic
    @State private var tapToAdd = false
    @State private var tracking = false
    @State private var didInitialCenter = false
    @State private var cameraHeading: Double = 0     // heading of the map camera, to orient the robot arrow
    @State private var trackHeading: Double = 0      // last heading applied while tracking
    @State private var trackCenter: CLLocationCoordinate2D?

    // Map zoom, as camera distance in metres. Tune these to taste.
    private static let fitDistance: CLLocationDistance = 150               // Fit button and Track mode
    private static let startDistance: CLLocationDistance = fitDistance * 2 // first view, a bit wider than Fit
    private static let trackPitch: Double = 0                              // 0 = straight down; try 45 for a chase view

    private func fitOnRobot() {
        tracking = false
        guard let c = store.status.gps.coordinate else {
            withAnimation { camera = .automatic }   // no fix yet: fit whatever is on the map
            return
        }
        withAnimation(.easeInOut(duration: 0.6)) {
            camera = .camera(MapCamera(centerCoordinate: c, distance: Self.fitDistance, heading: 0, pitch: 0))
        }
    }

    private func toggleTracking() {
        Haptics.light()
        if tracking {
            tracking = false
            return
        }
        guard store.status.gps.coordinate != nil else {
            store.showToast("The robot has no GPS fix.")
            return
        }
        tracking = true
        applyTrack(force: true)
    }

    /// Keeps the camera on the robot at fit zoom, rotated so the robot's compass heading points up.
    private func applyTrack(force: Bool = false) {
        guard tracking, let c = store.status.gps.coordinate else { return }
        let h = store.status.gps.heading ?? trackHeading
        if !force, let last = trackCenter,
           abs(Self.angleDelta(h, trackHeading)) < 3,
           CLLocation(latitude: c.latitude, longitude: c.longitude)
               .distance(from: CLLocation(latitude: last.latitude, longitude: last.longitude)) < 0.5 {
            return   // ignore compass jitter and GPS noise
        }
        trackHeading = h
        trackCenter = c
        withAnimation(.easeInOut(duration: 0.9)) {
            camera = .camera(MapCamera(centerCoordinate: c, distance: Self.fitDistance,
                                       heading: h, pitch: Self.trackPitch))
        }
    }

    private static func angleDelta(_ a: Double, _ b: Double) -> Double {
        var d = (a - b).truncatingRemainder(dividingBy: 360)
        if d > 180 { d -= 360 }
        if d < -180 { d += 360 }
        return d
    }

    var body: some View {
        let points: [MapPoint] = store.waypoints.enumerated().compactMap { item in
            guard let c = item.element.coordinate else { return nil }
            return MapPoint(id: item.offset, coordinate: c)
        }
        let robot = store.status.gps.coordinate
        let active = store.waypointsDirty ? -1 : store.status.wpIndex
        // Arrow angle on screen: straight up while tracking, else robot heading minus map rotation.
        let pinRotation: Double? = store.status.gps.heading.map { tracking ? 0 : $0 - cameraHeading }

        ApexCard("Route Map", info: """
            Robot position (arrow) and your route as it is in the editor below. The map opens on the \
            robot. Fit centers on the robot, north up. Track follows the robot at the same zoom and \
            rotates the map so the robot's compass heading points up; dragging the map while tracking \
            snaps back, so turn Track off to look around. Turn on Tap to add and tap the map to drop \
            a waypoint. Map imagery needs internet, so on the APEX hotspot the background may be \
            blank, but the points and route still draw.
            """) {
            MapReader { proxy in
                Map(position: $camera) {
                    if points.count >= 2 {
                        MapPolyline(coordinates: points.map(\.coordinate))
                            .stroke(Color.apexRedHi, style: StrokeStyle(lineWidth: 3, dash: [8, 5]))
                    }
                    ForEach(points) { item in
                        Annotation("", coordinate: item.coordinate, anchor: .center) {
                            WaypointPin(number: item.id + 1, active: item.id == active)
                        }
                    }
                    if let robot {
                        Annotation("APEX", coordinate: robot, anchor: .center) {
                            RobotPin(rotation: pinRotation)
                        }
                    }
                    UserAnnotation()
                }
                .mapStyle(.hybrid(elevation: .flat))
                .mapControls {
                    MapCompass()
                    MapScaleView()
                }
                .onMapCameraChange(frequency: .onEnd) { context in
                    cameraHeading = context.camera.heading
                }
                .onTapGesture(coordinateSpace: .local) { pt in
                    guard tapToAdd, let c = proxy.convert(pt, from: .local) else { return }
                    store.addWaypoint(c)
                }
            }
            .frame(height: 280)
            .clipShape(RoundedRectangle(cornerRadius: 10, style: .continuous))
            .overlay(RoundedRectangle(cornerRadius: 10, style: .continuous).strokeBorder(Color.apexLine, lineWidth: 1))
            .onChange(of: store.status.gps.hasFix, initial: true) { _, hasFix in
                // First time the robot has a fix, open the map on it, wider than Fit.
                guard hasFix, !didInitialCenter, let c = store.status.gps.coordinate else { return }
                didInitialCenter = true
                camera = .camera(MapCamera(centerCoordinate: c, distance: Self.startDistance, heading: 0, pitch: 0))
            }
            .onChange(of: store.status.gps) { _, _ in
                applyTrack()
            }

            HStack(spacing: 8) {
                Button {
                    tapToAdd.toggle()
                    Haptics.light()
                } label: {
                    Label(tapToAdd ? "Add: on" : "Tap to add", systemImage: "hand.tap.fill")
                }
                .apexButton(tapToAdd ? .on : .normal)

                Button {
                    Haptics.light()
                    fitOnRobot()
                } label: {
                    Label("Fit", systemImage: "scope")
                }
                .apexButton()

                Button {
                    toggleTracking()
                } label: {
                    Label(tracking ? "Tracking" : "Track", systemImage: "location.north.line.fill")
                }
                .apexButton(tracking ? .on : .normal)
            }
        }
    }
}

/// A waypoint that parses to a valid coordinate, keyed by its row index.
struct MapPoint: Identifiable {
    let id: Int
    let coordinate: CLLocationCoordinate2D
}

struct WaypointPin: View {
    let number: Int
    let active: Bool

    var body: some View {
        Text("\(number)")
            .font(.system(size: 12, weight: .heavy, design: .monospaced))
            .foregroundStyle(.white)
            .frame(width: 26, height: 26)
            .background(Circle().fill(active ? Color.apexRedHi : Color(red: 30 / 255, green: 30 / 255, blue: 36 / 255)))
            .overlay(Circle().strokeBorder(active ? Color.white : Color.apexRedHi, lineWidth: 2))
            .shadow(color: active ? Color.apexRed.opacity(0.8) : .black.opacity(0.5), radius: 5)
    }
}

struct RobotPin: View {
    /// Arrow angle on screen in degrees (0 = up). nil means no compass, so a plain dot.
    let rotation: Double?

    var body: some View {
        ZStack {
            Circle().fill(Color.apexRed.opacity(0.25)).frame(width: 40, height: 40)
            Image(systemName: rotation == nil ? "circle.fill" : "location.north.fill")
                .font(.system(size: rotation == nil ? 14 : 20, weight: .bold))
                .foregroundStyle(.white)
                .shadow(color: Color.apexRed, radius: 4)
                .rotationEffect(.degrees(rotation ?? 0))
        }
    }
}

// MARK: Nav mode + transport

@MainActor
struct NavModeCard: View {
    @Environment(RobotStore.self) private var store

    var body: some View {
        let s = store.status
        let live = store.isConnected
        ApexCard("Autonomous Navigation", info: """
            NAV MODE is the master autonomous switch. Start runs the route from the first point, \
            Pause holds position, Stop returns to manual. With no route loaded the robot just marches \
            in place. Start and Pause need the robot to be walking.
            """, chip: s.navOn ? (s.isNavPaused ? "PAUSED" : "ON") : "OFF", chipLive: s.navOn) {
            Button {
                store.toggleNav()
            } label: {
                Text(s.navOn ? "NAV MODE: ON" : (s.hasRoute ? "NAV MODE: OFF" : "NAV MODE: add a waypoint first"))
            }
            .apexButton(s.navOn ? .on : .off, tall: true)
            .disabled(!live || (!s.navOn && !s.hasRoute))

            HStack(spacing: 8) {
                Button(s.isNavPaused ? "Resume" : "Start") { store.navControl("start") }
                    .apexButton(.go)
                    .disabled(!live || !(s.isWalking && s.hasRoute && (!s.navOn || s.isNavPaused)))
                Button("Pause") { store.navControl("pause") }
                    .apexButton(.hold)
                    .disabled(!live || !(s.isWalking && s.navOn && !s.isNavPaused))
                Button("Stop") { store.navControl("stop") }
                    .apexButton(.stop)
                    .disabled(!live || !s.navOn)
            }

            Text(s.routeMessage)
                .font(.system(size: 12, design: .monospaced))
                .foregroundStyle(Color.apexMuted)
                .fixedSize(horizontal: false, vertical: true)
        }
    }
}

// MARK: Route editor

@MainActor
struct WaypointEditorCard: View {
    @Environment(RobotStore.self) private var store
    @FocusState private var focused: FieldID?

    enum FieldID: Hashable {
        case lat(UUID)
        case lon(UUID)
    }

    var body: some View {
        let active = store.waypointsDirty ? -1 : store.status.wpIndex
        ApexCard("Route Editor", info: """
            Latitude/longitude in decimal degrees (for example 41.0561, -74.1452). South and west are \
            negative. Paste a Google Maps "lat, lng" into either box and it splits across both. \
            Nothing reaches the robot until Send route.
            """, chip: "\(store.waypoints.count) PTS") {
            if store.waypoints.isEmpty {
                Text("No waypoints yet.")
                    .font(.system(size: 13))
                    .foregroundStyle(Color.apexMuted)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 6)
            }

            ForEach(Array(store.waypoints.enumerated()), id: \.element.id) { i, w in
                WaypointRow(index: i, draft: w, active: i == active,
                            isFirst: i == 0, isLast: i == store.waypoints.count - 1,
                            focused: $focused)
            }

            HStack(spacing: 8) {
                Button { store.addBlankWaypoint() } label: { Label("Point", systemImage: "plus") }
                    .apexButton()
                Button { store.addRobotPosition() } label: { Label("Robot", systemImage: "dot.scope") }
                    .apexButton()
                    .disabled(!store.status.gps.hasFix)
                Button { store.addPhonePosition() } label: { Label("Me", systemImage: "location.fill") }
                    .apexButton()
            }
            .labelStyle(.titleAndIcon)

            HStack(spacing: 8) {
                if store.waypointsDirty {
                    Button("Discard") { store.discardRouteEdits() }
                        .apexButton(.off)
                        .disabled(!store.isConnected)
                }
                Button {
                    focused = nil
                    store.sendRoute()
                } label: {
                    if store.sendingRoute { ProgressView().tint(.white) } else { Text("Send route") }
                }
                .apexButton(.primary, tall: true)
                .disabled(!store.isConnected || !store.waypointsDirty || store.sendingRoute)
            }

            if let note = store.routeNote {
                Text(note)
                    .font(.system(size: 12))
                    .foregroundStyle(store.waypointsDirty ? Color.apexAmber : Color.apexMuted)
            }
        }
        .toolbar {
            ToolbarItemGroup(placement: .keyboard) {
                Spacer()
                Button("Done") { focused = nil }
            }
        }
    }
}

@MainActor
struct WaypointRow: View {
    @Environment(RobotStore.self) private var store
    let index: Int
    let draft: WaypointDraft
    let active: Bool
    let isFirst: Bool
    let isLast: Bool
    var focused: FocusState<WaypointEditorCard.FieldID?>.Binding

    init(index: Int, draft: WaypointDraft, active: Bool, isFirst: Bool, isLast: Bool,
         focused: FocusState<WaypointEditorCard.FieldID?>.Binding) {
        self.index = index
        self.draft = draft
        self.active = active
        self.isFirst = isFirst
        self.isLast = isLast
        self.focused = focused
    }

    var body: some View {
        HStack(spacing: 6) {
            Text("\(index + 1)")
                .font(.system(size: 13, weight: active ? .heavy : .regular, design: .monospaced))
                .foregroundStyle(active ? Color.apexRedHi : Color.apexMuted)
                .frame(width: 20, alignment: .trailing)

            field("lat", text: draft.lat, bad: draft.latBad, latitude: true)
                .focused(focused, equals: .lat(draft.id))
            field("lon", text: draft.lon, bad: draft.lonBad, latitude: false)
                .focused(focused, equals: .lon(draft.id))

            Menu {
                Button { store.moveWaypoint(draft.id, by: -1) } label: { Label("Move up", systemImage: "arrow.up") }
                    .disabled(isFirst)
                Button { store.moveWaypoint(draft.id, by: 1) } label: { Label("Move down", systemImage: "arrow.down") }
                    .disabled(isLast)
                Button(role: .destructive) { store.removeWaypoint(draft.id) } label: {
                    Label("Remove", systemImage: "trash")
                }
            } label: {
                Image(systemName: "ellipsis")
                    .font(.system(size: 14, weight: .bold))
                    .foregroundStyle(Color.apexText)
                    .frame(width: 34, height: 36)
                    .background(Color(red: 23 / 255, green: 23 / 255, blue: 27 / 255), in: RoundedRectangle(cornerRadius: 7))
                    .overlay(RoundedRectangle(cornerRadius: 7).strokeBorder(Color.apexLine, lineWidth: 1))
            }
            .accessibilityLabel("Waypoint \(index + 1) options")
        }
    }

    private func field(_ placeholder: String, text: String, bad: Bool, latitude: Bool) -> some View {
        TextField(placeholder, text: Binding(
            get: { text },
            set: { store.editWaypoint(draft.id, latitude: latitude, value: $0) }))
            .keyboardType(.numbersAndPunctuation)   // decimal pad has no minus sign
            .autocorrectionDisabled()
            .textInputAutocapitalization(.never)
            .font(.system(size: 13, design: .monospaced))
            .foregroundStyle(Color.apexText)
            .padding(.horizontal, 8)
            .frame(height: 36)
            .background(Color.apexWell, in: RoundedRectangle(cornerRadius: 7))
            .overlay(RoundedRectangle(cornerRadius: 7)
                .strokeBorder(bad ? Color.apexAmber : Color(red: 46 / 255, green: 46 / 255, blue: 54 / 255), lineWidth: 1))
    }
}
