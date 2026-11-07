import Foundation

/// Decoded GET /api/state (schema 1). Every field is optional because the Pi sends
/// null for anything it does not have, and newer schemas may add or drop keys.
struct ApexState: Decodable, Equatable {
    struct Meta: Decodable, Equatable {
        var schema: Int?
        var time_unix: Double?
        var uptime_s: Double?
    }
    struct Robot: Decodable, Equatable {
        var all_homed: Bool?
        var standing: Bool?
        var walking: Bool?
        var steering_deg: Double?
    }
    struct Side: Decodable, Equatable {
        var right: Double?
        var left: Double?
    }
    struct Currents: Decodable, Equatable {
        var roll: Side?
        var pitch: Side?
        var knee: Side?
    }
    struct StateLeg: Decodable, Equatable {
        var name: String?
        var index: Int?
        var homed: Bool?
        var deactivated: Bool?
        var currents_ma: Currents?
    }
    struct Subsystems: Decodable, Equatable {
        var imu: Bool?
        var gps: Bool?
        var compass: Bool?
        var power: Bool?
        var camera: Bool?
        var audio: Bool?
    }
    struct Health: Decodable, Equatable {
        var ok: Bool?
        var offline: [String]?
        var subsystems: Subsystems?
    }
    struct GPS: Decodable, Equatable {
        var fix: Bool?
        var satellites: Int?
        var lat: Double?
        var lon: Double?
        var heading_deg: Double?
        var compass_point: String?
    }
    struct Power: Decodable, Equatable {
        var valid: Bool?
        var rail_5v_amps: Double?
        var rail_5v_volts: Double?
        var rail_5v_watts: Double?
    }
    struct Navigation: Decodable, Equatable {
        var autonomous: Bool?
        var paused: Bool?
        var waypoint_index: Int?
        var waypoint_total: Int?
        var route_complete: Bool?
    }
    struct Avoidance: Decodable, Equatable {
        var available: Bool?
        var enabled: Bool?
        var state: String?
        var steer_deg: Double?
        var stride_pct: Double?
    }
    struct Camera: Decodable, Equatable {
        var frame_age_s: Double?
        var live: Bool?
    }

    var meta: Meta?
    var robot: Robot?
    var legs: [StateLeg]?
    var health: Health?
    var gps: GPS?
    var power: Power?
    var navigation: Navigation?
    var avoidance: Avoidance?
    var camera: Camera?
}
