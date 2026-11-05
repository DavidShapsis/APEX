import Foundation
import CoreLocation
import SwiftUI

// Mirrors the JSON that stream_server.py returns from GET /status.
// Every field is optional on the wire, so an older controller that sends a
// shorter status array still decodes (same defaults stream_server.py uses).

struct GPSInfo: Decodable, Equatable {
    var fix: Int = 0
    var sats: Int = 0
    var lat: Double? = nil
    var lon: Double? = nil
    var heading: Double? = nil

    init() {}

    enum CodingKeys: String, CodingKey { case fix, sats, lat, lon, heading }

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        fix = try c.decodeIfPresent(Int.self, forKey: .fix) ?? 0
        sats = try c.decodeIfPresent(Int.self, forKey: .sats) ?? 0
        lat = try c.decodeIfPresent(Double.self, forKey: .lat)
        lon = try c.decodeIfPresent(Double.self, forKey: .lon)
        heading = try c.decodeIfPresent(Double.self, forKey: .heading)
    }

    var hasFix: Bool { fix == 1 && lat != nil && lon != nil }

    var coordinate: CLLocationCoordinate2D? {
        guard hasFix, let lat, let lon else { return nil }
        return CLLocationCoordinate2D(latitude: lat, longitude: lon)
    }

    var compassPoint: String? {
        guard let heading else { return nil }
        let pts = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
        let i = Int((heading / 45).rounded()) % 8
        return pts[(i + 8) % 8]
    }
}

struct ApexStatus: Decodable, Equatable {
    var homed: [Int] = [0, 0, 0, 0]
    var standing: Int = 0
    var walking: Int = 0
    var deactivated: [Int] = [0, 0, 0, 0]
    var avoidAvailable: Int = 0
    var avoidEnabled: Int = 0
    var avoidState: Int = 0
    var avoidSteer: Int = 0
    var avoidStride: Int = 100
    var wpIndex: Int = 0
    var wpTotal: Int = 0
    var navMode: Int = 0
    var navPaused: Int = 0
    var health: Int = 0x3F
    var currents: [Int] = Array(repeating: -1, count: 24)
    var gps = GPSInfo()

    init() {}

    enum CodingKeys: String, CodingKey {
        case homed, standing, walking, deactivated, health, currents, gps
        case avoidAvailable = "avoid_available"
        case avoidEnabled = "avoid_enabled"
        case avoidState = "avoid_state"
        case avoidSteer = "avoid_steer"
        case avoidStride = "avoid_stride"
        case wpIndex = "wp_index"
        case wpTotal = "wp_total"
        case navMode = "nav_mode"
        case navPaused = "nav_paused"
    }

    init(from decoder: Decoder) throws {
        let c = try decoder.container(keyedBy: CodingKeys.self)
        let d = ApexStatus()
        homed = try c.decodeIfPresent([Int].self, forKey: .homed) ?? d.homed
        standing = try c.decodeIfPresent(Int.self, forKey: .standing) ?? d.standing
        walking = try c.decodeIfPresent(Int.self, forKey: .walking) ?? d.walking
        deactivated = try c.decodeIfPresent([Int].self, forKey: .deactivated) ?? d.deactivated
        avoidAvailable = try c.decodeIfPresent(Int.self, forKey: .avoidAvailable) ?? d.avoidAvailable
        avoidEnabled = try c.decodeIfPresent(Int.self, forKey: .avoidEnabled) ?? d.avoidEnabled
        avoidState = try c.decodeIfPresent(Int.self, forKey: .avoidState) ?? d.avoidState
        avoidSteer = try c.decodeIfPresent(Int.self, forKey: .avoidSteer) ?? d.avoidSteer
        avoidStride = try c.decodeIfPresent(Int.self, forKey: .avoidStride) ?? d.avoidStride
        wpIndex = try c.decodeIfPresent(Int.self, forKey: .wpIndex) ?? d.wpIndex
        wpTotal = try c.decodeIfPresent(Int.self, forKey: .wpTotal) ?? d.wpTotal
        navMode = try c.decodeIfPresent(Int.self, forKey: .navMode) ?? d.navMode
        navPaused = try c.decodeIfPresent(Int.self, forKey: .navPaused) ?? d.navPaused
        health = try c.decodeIfPresent(Int.self, forKey: .health) ?? d.health
        currents = try c.decodeIfPresent([Int].self, forKey: .currents) ?? d.currents
        gps = try c.decodeIfPresent(GPSInfo.self, forKey: .gps) ?? d.gps

        // Pad short arrays so index lookups in the UI never trap.
        while homed.count < 4 { homed.append(0) }
        while deactivated.count < 4 { deactivated.append(0) }
        while currents.count < 24 { currents.append(-1) }
    }

    // MARK: Derived state (same rules as dashboard_page.py refreshStatus)

    var homedCount: Int { homed.prefix(4).filter { $0 == 1 }.count }
    var allHomed: Bool { homedCount == 4 }
    var isStanding: Bool { standing == 1 }
    var isWalking: Bool { walking == 1 }
    var navOn: Bool { navMode == 1 }
    var isNavPaused: Bool { navPaused == 1 }
    var hasRoute: Bool { wpTotal > 0 }
    var avoidIsAvailable: Bool { avoidAvailable == 1 }
    var avoidIsOn: Bool { avoidEnabled == 1 }

    func isHomed(_ leg: Int) -> Bool { homed.indices.contains(leg) && homed[leg] == 1 }
    func isDeactivated(_ leg: Int) -> Bool { deactivated.indices.contains(leg) && deactivated[leg] == 1 }

    /// Subsystems whose health bit is clear. Bit order matches HEALTH_BITS in pi5_main.py.
    var offlineSubsystems: [String] {
        Subsystem.all.enumerated().compactMap { i, name in ((health >> i) & 1) == 0 ? name : nil }
    }

    func subsystemUp(_ index: Int) -> Bool { ((health >> index) & 1) == 1 }

    /// Current in mA for [leg][joint][side], or nil when that channel has no data (-1).
    /// side 0 = R (RPWM), 1 = L (LPWM).
    func current(leg: Int, joint: Int, side: Int) -> Int? {
        let i = leg * 6 + joint * 2 + side
        guard currents.indices.contains(i), currents[i] >= 0 else { return nil }
        return currents[i]
    }

    func legReportsCurrent(_ leg: Int) -> Bool {
        (0..<3).contains { current(leg: leg, joint: $0, side: 0) != nil && current(leg: leg, joint: $0, side: 1) != nil }
    }

    var legsReportingCurrent: Int { (0..<4).filter { legReportsCurrent($0) }.count }

    var routeMessage: String {
        var msg: String
        if wpTotal == 0 {
            msg = "No route. Add at least one waypoint to enable NAV MODE."
        } else if wpIndex >= wpTotal {
            msg = "Route complete (\(wpTotal)/\(wpTotal)). Holding position."
        } else {
            msg = "Driving to waypoint \(wpIndex + 1) of \(wpTotal)"
        }
        if isNavPaused { msg = "PAUSED: " + msg } else if !navOn { msg = "Nav off: " + msg }
        return msg
    }
}

enum Leg {
    static let names = ["FL", "FR", "RR", "RL"]
}

enum Joint {
    static let names = ["Roll", "Pitch", "Knee"]
}

enum Subsystem {
    static let all = ["IMU", "GPS", "Compass", "Power monitor", "Camera", "Audio"]
    static let icons = ["gyroscope", "location.fill", "safari", "bolt.fill", "camera.fill", "speaker.wave.2.fill"]
}

/// Must match AvoidState.CODES in vision_obstacle.py.
struct AvoidStateInfo {
    let label: String
    let color: Color
    let detail: String

    static func from(_ code: Int) -> AvoidStateInfo {
        switch code {
        case 1: return .init(label: "CLEAR", color: .apexGreen, detail: "Path clear, navigation steering")
        case 2: return .init(label: "AVOIDING", color: .apexAmber, detail: "Obstacle ahead, committed to a detour")
        case 3: return .init(label: "CLEARING", color: .apexAmber, detail: "Driving past before turning back")
        case 4: return .init(label: "BLOCKED", color: .apexBad, detail: "No gap, marching in place")
        case 5: return .init(label: "ESCAPE", color: .apexBad, detail: "Arcing to find a way out")
        default: return .init(label: "OFF", color: .apexMuted, detail: "Not running")
        }
    }
}

/// One row of the route editor. Text, not numbers, while the operator types.
struct WaypointDraft: Identifiable, Equatable {
    var id = UUID()
    var lat: String
    var lon: String

    var latValue: Double? {
        guard let v = Double(lat.trimmingCharacters(in: .whitespaces)), (-90.0...90.0).contains(v) else { return nil }
        return v
    }

    var lonValue: Double? {
        guard let v = Double(lon.trimmingCharacters(in: .whitespaces)), (-180.0...180.0).contains(v) else { return nil }
        return v
    }

    var isValid: Bool { latValue != nil && lonValue != nil }
    var latBad: Bool { !lat.isEmpty && latValue == nil }
    var lonBad: Bool { !lon.isEmpty && lonValue == nil }

    var coordinate: CLLocationCoordinate2D? {
        guard let a = latValue, let o = lonValue else { return nil }
        return CLLocationCoordinate2D(latitude: a, longitude: o)
    }

    init(lat: String, lon: String) {
        self.lat = lat
        self.lon = lon
    }

    init(_ c: CLLocationCoordinate2D) {
        self.lat = String(format: "%.7f", c.latitude)
        self.lon = String(format: "%.7f", c.longitude)
    }

    /// Google Maps "copy coordinates" gives "lat, lon" as one string.
    /// Returns the two halves when a pasted value looks like that.
    static func splitPair(_ s: String) -> (String, String)? {
        let parts = s.split(whereSeparator: { $0 == "," || $0 == " " || $0 == "\t" }).map(String.init)
        guard parts.count == 2, Double(parts[0]) != nil, Double(parts[1]) != nil else { return nil }
        return (parts[0], parts[1])
    }
}

enum ConnectionState: Equatable {
    case connecting
    case connected
    case disconnected
}
