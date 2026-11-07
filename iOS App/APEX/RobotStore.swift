import Foundation
import CoreLocation
import Observation

/// App-wide state: polls /status once a second (like the web dashboard),
/// sends commands, and holds the route editor's working copy.
@Observable
@MainActor
final class RobotStore {
    static let defaultHost = "apex.local"
    private static let hostKey = "robotHost"

    private(set) var host: String
    private(set) var client: ApexClient?
    private(set) var status = ApexStatus()
    private(set) var connection: ConnectionState = .connecting
    private(set) var lastUpdate: Date?
    private(set) var lastError: String?

    /// Latest /api/state, and whether the Pi answered 404 (route not installed yet).
    private(set) var state: ApexState?
    private(set) var stateMissing = false

    let feed: MJPEGStream
    let capture: CaptureController

    /// Last steering angle sent. The robot does not report it back in /status,
    /// so like the web page this starts at 0 on launch.
    private(set) var direction = 0

    // Route editor working copy. Nothing reaches the robot until sendRoute().
    var waypoints: [WaypointDraft] = []
    private(set) var waypointsDirty = false
    var routeNote: String?
    private(set) var sendingRoute = false
    private var waypointsLoaded = false

    var toast: String?
    private(set) var toastSuccess = false
    private var toastTask: Task<Void, Never>?

    private var pollTask: Task<Void, Never>?
    private var failures = 0
    private var steerInFlight = false
    private var steerPending: Int?
    private let location = LocationProvider()

    // When the address is apex.local (with or without a :port), the app also tries
    // robodog.local, the Pi's hostname. A name that answered is remembered and tried first.
    private static let lastGoodKey = "lastGoodHost"
    private static let autoNames = ["apex.local", "robodog.local"]
    private var hostCandidates: [String] = []
    private var candidateIndex = 0
    private var hostEverWorked = false

    init() {
        let stream = MJPEGStream()
        feed = stream
        capture = CaptureController(feed: stream)
        let saved = UserDefaults.standard.string(forKey: Self.hostKey) ?? Self.defaultHost
        host = saved
        hostCandidates = Self.makeCandidates(for: saved)
        useCandidate(0)
        capture.onMessage = { [weak self] message, success in
            self?.showToast(message, success: success)
        }
    }

    var isConnected: Bool { connection == .connected }

    /// The address actually in use right now (may be robodog.local in automatic mode).
    var activeHost: String {
        hostCandidates.indices.contains(candidateIndex) ? hostCandidates[candidateIndex] : host
    }

    private static func makeCandidates(for host: String) -> [String] {
        let trimmed = host.trimmingCharacters(in: .whitespaces)
        let parts = trimmed.split(separator: ":", maxSplits: 1, omittingEmptySubsequences: false)
        guard let first = parts.first, first.lowercased() == defaultHost else { return [trimmed] }
        let suffix = parts.count > 1 ? ":" + parts[1] : ""
        var list = autoNames.map { $0 + suffix }
        if let good = UserDefaults.standard.string(forKey: lastGoodKey),
           let i = list.firstIndex(of: good), i != 0 {
            list.remove(at: i)
            list.insert(good, at: 0)
        }
        return list
    }

    private func useCandidate(_ index: Int) {
        candidateIndex = index
        hostEverWorked = false
        waypointsLoaded = false
        let h = hostCandidates.indices.contains(index) ? hostCandidates[index] : host
        client = ApexClient.makeBaseURL(from: h).map { ApexClient(baseURL: $0) }
        feed.setURL(client?.videoURL)
    }

    // MARK: Lifecycle

    func setHost(_ raw: String) {
        let trimmed = raw.trimmingCharacters(in: .whitespacesAndNewlines)
        let value = trimmed.isEmpty ? Self.defaultHost : trimmed
        guard ApexClient.makeBaseURL(from: value) != nil else {
            showToast("\"\(value)\" is not a valid address")
            return
        }
        host = value
        UserDefaults.standard.set(value, forKey: Self.hostKey)
        hostCandidates = Self.makeCandidates(for: value)
        status = ApexStatus()
        state = nil
        stateMissing = false
        connection = .connecting
        lastUpdate = nil
        failures = 0
        useCandidate(0)
        Task { await refresh() }
    }

    /// Opens iOS Wi-Fi settings so the phone can join the robot's network. The app keeps
    /// polling, so it reconnects by itself once the phone is back on the robot's Wi-Fi.
    func openWiFiSettings() {
        showToast("In Settings, open Wi-Fi and join APEXWifi")
        SettingsOpener.openWiFi()
    }

    func setActive(_ active: Bool) {
        feed.setSuspended(!active)
        if active { startPolling() } else { stopPolling() }
    }

    private func startPolling() {
        guard pollTask == nil else { return }
        pollTask = Task { [weak self] in
            while !Task.isCancelled {
                await self?.refresh()
                try? await Task.sleep(for: .seconds(1))
            }
        }
    }

    private func stopPolling() {
        pollTask?.cancel()
        pollTask = nil
    }

    func refresh() async {
        guard let client else {
            connection = .disconnected
            return
        }
        do {
            let s = try await client.status()
            guard client.baseURL == self.client?.baseURL else { return }   // host changed mid-request
            status = s
            lastUpdate = Date()
            lastError = nil
            failures = 0
            connection = .connected
            hostEverWorked = true
            if hostCandidates.count > 1 {
                let active = activeHost
                if UserDefaults.standard.string(forKey: Self.lastGoodKey) != active {
                    UserDefaults.standard.set(active, forKey: Self.lastGoodKey)
                }
            }
            if !waypointsLoaded { await loadWaypoints() }
            await refreshState(client)
        } catch {
            guard client.baseURL == self.client?.baseURL else { return }
            failures += 1
            lastError = error.localizedDescription
            if failures >= 2 {
                connection = .disconnected
                // Never reached this name yet: try the next one.
                if hostCandidates.count > 1 && !hostEverWorked {
                    failures = 0
                    useCandidate((candidateIndex + 1) % hostCandidates.count)
                }
            }
        }
    }

    /// Sensor readout. Failure here never affects the connection light.
    private func refreshState(_ client: ApexClient) async {
        do {
            let s = try await client.state()
            guard client.baseURL == self.client?.baseURL else { return }
            state = s
            stateMissing = false
        } catch ApexError.http(404) {
            stateMissing = true
            state = nil
        } catch {
            // Keep the last values; the next poll retries.
        }
    }

    // MARK: Commands

    /// Sends one command, surfaces the robot's error text, then refreshes so
    /// the UI shows what the controller actually did.
    private func command(_ path: String, form: [String: String] = [:]) {
        guard let client else { return }
        Haptics.tap()
        Task {
            do {
                let r = try await client.post(path, form: form)
                if !r.ok {
                    Haptics.error()
                    showToast(r.error ?? "The robot rejected that command.")
                }
            } catch {
                Haptics.error()
                showToast("Could not reach the robot.")
            }
            await refresh()
        }
    }

    func homeLeg(_ leg: Int) { command("/home_leg", form: ["leg": String(leg)]) }
    func stand() { command("/stand") }
    func go() { command("/go") }
    func stopMotors() { command("/stop") }
    func toggleNav() { command("/toggle_nav") }
    func toggleAvoid() { command("/toggle_avoid") }
    func navControl(_ action: String) { command("/nav_control", form: ["action": action]) }

    func toggleLeg(_ leg: Int) {
        let path = status.isDeactivated(leg) ? "/reactivate_leg" : "/deactivate_leg"
        command(path, form: ["leg": String(leg)])
    }

    // MARK: Steering

    /// Coalesces slider drags: at most one request in flight, and the newest
    /// angle always goes out last.
    func setDirection(_ angle: Int) {
        let a = max(-90, min(90, angle))
        direction = a
        if steerInFlight {
            steerPending = a
        } else {
            sendSteer(a)
        }
    }

    private func sendSteer(_ a: Int) {
        guard let client else { return }
        steerInFlight = true
        Task {
            _ = try? await client.post("/set_direction", form: ["angle": String(a)])
            steerInFlight = false
            if let next = steerPending {
                steerPending = nil
                if next != a { sendSteer(next) }
            }
        }
    }

    // MARK: Route editor

    func loadWaypoints() async {
        guard let client, !waypointsDirty else { return }
        do {
            let pts = try await client.waypoints()
            guard !waypointsDirty else { return }   // do not stomp on typing
            waypoints = pts.compactMap { p in
                guard p.count >= 2 else { return nil }
                return WaypointDraft(lat: String(p[0]), lon: String(p[1]))
            }
            waypointsLoaded = true
        } catch {
            // Try again on the next status poll.
        }
    }

    func editWaypoint(_ id: UUID, latitude: Bool, value: String) {
        guard let i = waypoints.firstIndex(where: { $0.id == id }) else { return }
        let v = value.trimmingCharacters(in: .whitespaces)
        if let pair = WaypointDraft.splitPair(v) {
            waypoints[i].lat = pair.0
            waypoints[i].lon = pair.1
        } else if latitude {
            waypoints[i].lat = v
        } else {
            waypoints[i].lon = v
        }
        markDirty()
    }

    func addBlankWaypoint() {
        waypoints.append(WaypointDraft(lat: "", lon: ""))
        markDirty()
    }

    func addWaypoint(_ c: CLLocationCoordinate2D) {
        waypoints.append(WaypointDraft(c))
        markDirty()
        Haptics.tap()
    }

    func addRobotPosition() {
        guard let c = status.gps.coordinate else {
            showToast("The robot has no GPS fix.")
            return
        }
        addWaypoint(c)
    }

    func addPhonePosition() {
        Task {
            do {
                let c = try await location.requestOnce()
                addWaypoint(c)
            } catch {
                showToast(error.localizedDescription)
            }
        }
    }

    func removeWaypoint(_ id: UUID) {
        waypoints.removeAll { $0.id == id }
        markDirty()
    }

    func moveWaypoint(_ id: UUID, by delta: Int) {
        guard let i = waypoints.firstIndex(where: { $0.id == id }) else { return }
        let j = i + delta
        guard waypoints.indices.contains(j) else { return }
        waypoints.swapAt(i, j)
        markDirty()
    }

    func discardRouteEdits() {
        waypointsDirty = false
        routeNote = nil
        Task { await loadWaypoints() }
    }

    private func markDirty() {
        waypointsDirty = true
        routeNote = "Unsent changes. Press Send route."
    }

    func sendRoute() {
        guard let client else { return }
        var out: [[Double]] = []
        for (k, w) in waypoints.enumerated() {
            guard let a = w.latValue, let o = w.lonValue else {
                routeNote = "Point \(k + 1) is not a valid latitude/longitude."
                Haptics.error()
                return
            }
            out.append([a, o])
        }
        sendingRoute = true
        Task {
            defer { sendingRoute = false }
            do {
                let r = try await client.setWaypoints(out)
                if r.ok {
                    waypointsDirty = false
                    let n = r.count ?? out.count
                    routeNote = "Route sent (\(n) point\(n == 1 ? "" : "s"))."
                    Haptics.success()
                } else {
                    routeNote = r.error ?? "Route rejected."
                    Haptics.error()
                }
            } catch {
                routeNote = "Could not reach the robot."
                Haptics.error()
            }
            await refresh()
        }
    }

    // MARK: Toast

    func showToast(_ message: String, success: Bool = false) {
        toastSuccess = success
        toast = message
        toastTask?.cancel()
        toastTask = Task { [weak self] in
            try? await Task.sleep(for: .seconds(3.5))
            guard !Task.isCancelled else { return }
            self?.toast = nil
        }
    }
}
