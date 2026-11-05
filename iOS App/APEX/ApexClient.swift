import Foundation

/// Result of a command endpoint. stream_server.py answers most POSTs with
/// {"ok": true} or {"ok": false, "error": "..."} (409/400), and /set_direction
/// with plain-text "OK".
struct CommandResult {
    let ok: Bool
    let error: String?
    let count: Int?
}

enum ApexError: LocalizedError {
    case badAddress
    case http(Int)

    var errorDescription: String? {
        switch self {
        case .badAddress: return "The robot address is not a valid URL."
        case .http(let code): return "The robot answered HTTP \(code)."
        }
    }
}

/// Thin async wrapper over the Flask routes in Code/Pi5/stream_server.py.
struct ApexClient: Sendable {
    let baseURL: URL
    private let session: URLSession

    init(baseURL: URL) {
        self.baseURL = baseURL
        let cfg = URLSessionConfiguration.ephemeral
        cfg.timeoutIntervalForRequest = 3
        cfg.timeoutIntervalForResource = 6
        cfg.waitsForConnectivity = false
        cfg.requestCachePolicy = .reloadIgnoringLocalCacheData
        cfg.httpMaximumConnectionsPerHost = 4
        self.session = URLSession(configuration: cfg)
    }

    /// Accepts "apex.local", "apex.local:5000", "192.168.4.1" or a full "http://..." URL.
    static func makeBaseURL(from raw: String) -> URL? {
        var s = raw.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !s.isEmpty else { return nil }
        if !s.lowercased().hasPrefix("http://") && !s.lowercased().hasPrefix("https://") {
            s = "http://" + s
        }
        while s.hasSuffix("/") { s.removeLast() }
        guard let url = URL(string: s), url.host != nil else { return nil }
        return url
    }

    func url(_ path: String) -> URL {
        baseURL.appending(path: String(path.drop(while: { $0 == "/" })))
    }

    var videoURL: URL { url("/video_feed") }

    // MARK: GET

    func status() async throws -> ApexStatus {
        let (data, response) = try await session.data(from: url("/status"))
        try Self.check(response)
        return try JSONDecoder().decode(ApexStatus.self, from: data)
    }

    func waypoints() async throws -> [[Double]] {
        struct Body: Decodable { let waypoints: [[Double]] }
        let (data, response) = try await session.data(from: url("/waypoints"))
        try Self.check(response)
        return try JSONDecoder().decode(Body.self, from: data).waypoints
    }

    // MARK: POST

    /// Form-encoded POST, matching the dashboard's fetch() calls.
    func post(_ path: String, form: [String: String] = [:]) async throws -> CommandResult {
        var req = URLRequest(url: url(path))
        req.httpMethod = "POST"
        if !form.isEmpty {
            var comps = URLComponents()
            comps.queryItems = form.map { URLQueryItem(name: $0.key, value: $0.value) }
            req.setValue("application/x-www-form-urlencoded", forHTTPHeaderField: "Content-Type")
            req.httpBody = (comps.percentEncodedQuery ?? "").data(using: .utf8)
        }
        let (data, response) = try await session.data(for: req)
        return try Self.parseCommand(data: data, response: response)
    }

    /// POST /set_waypoints with JSON {"waypoints": [[lat, lon], ...]}.
    func setWaypoints(_ points: [[Double]]) async throws -> CommandResult {
        var req = URLRequest(url: url("/set_waypoints"))
        req.httpMethod = "POST"
        req.setValue("application/json", forHTTPHeaderField: "Content-Type")
        req.httpBody = try JSONSerialization.data(withJSONObject: ["waypoints": points])
        let (data, response) = try await session.data(for: req)
        return try Self.parseCommand(data: data, response: response)
    }

    // MARK: Helpers

    private static func check(_ response: URLResponse) throws {
        if let http = response as? HTTPURLResponse, !(200..<300).contains(http.statusCode) {
            throw ApexError.http(http.statusCode)
        }
    }

    private static func parseCommand(data: Data, response: URLResponse) throws -> CommandResult {
        let code = (response as? HTTPURLResponse)?.statusCode ?? 200
        if let obj = try? JSONSerialization.jsonObject(with: data) as? [String: Any] {
            let ok = (obj["ok"] as? Bool) ?? (200..<300).contains(code)
            return CommandResult(ok: ok, error: obj["error"] as? String, count: obj["count"] as? Int)
        }
        if (200..<300).contains(code) {
            return CommandResult(ok: true, error: nil, count: nil)
        }
        throw ApexError.http(code)
    }
}
