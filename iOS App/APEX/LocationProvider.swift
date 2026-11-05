import Foundation
import CoreLocation

enum LocationError: LocalizedError {
    case denied
    case busy
    case failed

    var errorDescription: String? {
        switch self {
        case .denied: return "Location access is off for APEX. Turn it on in Settings > Privacy > Location Services."
        case .busy: return "Already getting your location."
        case .failed: return "Could not get your location."
        }
    }
}

/// One-shot phone location, used for "Add my location" in the route editor.
@MainActor
final class LocationProvider: NSObject, CLLocationManagerDelegate {
    private let manager = CLLocationManager()
    private var continuation: CheckedContinuation<CLLocationCoordinate2D, Error>?

    override init() {
        super.init()
        manager.delegate = self
        manager.desiredAccuracy = kCLLocationAccuracyBest
    }

    func requestOnce() async throws -> CLLocationCoordinate2D {
        guard continuation == nil else { throw LocationError.busy }
        switch manager.authorizationStatus {
        case .denied, .restricted:
            throw LocationError.denied
        default:
            break
        }
        return try await withCheckedThrowingContinuation { cont in
            continuation = cont
            if manager.authorizationStatus == .notDetermined {
                manager.requestWhenInUseAuthorization()
            } else {
                manager.requestLocation()
            }
        }
    }

    private func finish(_ result: Result<CLLocationCoordinate2D, Error>) {
        guard let c = continuation else { return }
        continuation = nil
        c.resume(with: result)
    }

    private func authorizationChanged(_ status: CLAuthorizationStatus) {
        guard continuation != nil else { return }
        switch status {
        case .authorizedWhenInUse, .authorizedAlways:
            manager.requestLocation()
        case .denied, .restricted:
            finish(.failure(LocationError.denied))
        default:
            break
        }
    }

    // MARK: CLLocationManagerDelegate (delivered on the main thread, where the manager was made)

    nonisolated func locationManagerDidChangeAuthorization(_ manager: CLLocationManager) {
        let status = manager.authorizationStatus
        Task { @MainActor in self.authorizationChanged(status) }
    }

    nonisolated func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        guard let c = locations.last?.coordinate else { return }
        Task { @MainActor in self.finish(.success(c)) }
    }

    nonisolated func locationManager(_ manager: CLLocationManager, didFailWithError error: Error) {
        Task { @MainActor in self.finish(.failure(LocationError.failed)) }
    }
}
