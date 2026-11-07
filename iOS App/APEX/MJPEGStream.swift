import Foundation
import UIKit
import Observation

/// Plays the robot's /video_feed (multipart/x-mixed-replace MJPEG from Flask).
/// Neither AVPlayer nor SwiftUI plays MJPEG, so this pulls the HTTP stream and
/// cuts JPEGs out of it by their SOI (FFD8) / EOI (FFD9) markers. That works
/// whether URLSession hands over raw multipart bytes or one part at a time.
@Observable
@MainActor
final class MJPEGStream {
    enum State: Equatable { case idle, connecting, streaming, failed }

    private(set) var image: UIImage?
    /// The newest frame exactly as the robot sent it (for lossless snapshots).
    @ObservationIgnored private(set) var latestJPEG: Data?
    /// Called on the main actor for every decoded frame (used by the video recorder).
    @ObservationIgnored var frameHandler: (@MainActor (UIImage) -> Void)?
    /// Called when streaming shuts down: app backgrounded, last viewer gone, or no address.
    @ObservationIgnored var onStopped: (@MainActor () -> Void)?
    private(set) var state: State = .idle
    private(set) var fps: Int = 0

    // Runs only while at least one view is showing it, the app is in the
    // foreground and an address is set.
    private var consumers = 0
    private var suspended = false
    private var url: URL?

    private var session: URLSession?
    private var generation = 0
    private var retryTask: Task<Void, Never>?
    private var framesThisSecond = 0
    private var fpsWindowStart = Date()

    func setURL(_ newURL: URL?) {
        guard newURL != url else { return }
        url = newURL
        restart()
    }

    func acquire() { consumers += 1; update() }
    func release() { consumers = max(0, consumers - 1); update() }
    func setSuspended(_ s: Bool) { suspended = s; update() }

    private var shouldRun: Bool { consumers > 0 && !suspended && url != nil }

    private func update() {
        if shouldRun {
            if session == nil && retryTask == nil { connect() }
        } else {
            teardown()
            state = .idle
            onStopped?()
        }
    }

    private func restart() {
        teardown()
        image = nil
        latestJPEG = nil
        update()
    }

    private func connect() {
        guard let url else { return }
        generation += 1
        let gen = generation
        state = .connecting

        let receiver = MJPEGReceiver(
            onFrame: { [weak self] img, jpeg in
                Task { @MainActor [weak self] in self?.handleFrame(img, jpeg: jpeg, gen: gen) }
            },
            onEnd: { [weak self] in
                Task { @MainActor [weak self] in self?.handleEnd(gen: gen) }
            })

        let cfg = URLSessionConfiguration.ephemeral
        cfg.timeoutIntervalForRequest = 10     // idle gap allowed between chunks
        cfg.requestCachePolicy = .reloadIgnoringLocalCacheData
        let queue = OperationQueue()
        queue.maxConcurrentOperationCount = 1
        queue.qualityOfService = .userInitiated
        let s = URLSession(configuration: cfg, delegate: receiver, delegateQueue: queue)
        session = s
        s.dataTask(with: url).resume()
    }

    private func teardown() {
        retryTask?.cancel()
        retryTask = nil
        session?.invalidateAndCancel()
        session = nil
        generation += 1
    }

    private func handleFrame(_ img: UIImage, jpeg: Data, gen: Int) {
        guard gen == generation else { return }
        image = img
        latestJPEG = jpeg
        state = .streaming
        frameHandler?(img)
        framesThisSecond += 1
        let now = Date()
        let dt = now.timeIntervalSince(fpsWindowStart)
        if dt >= 1 {
            fps = Int((Double(framesThisSecond) / dt).rounded())
            framesThisSecond = 0
            fpsWindowStart = now
        }
    }

    private func handleEnd(gen: Int) {
        guard gen == generation else { return }
        session?.invalidateAndCancel()
        session = nil
        state = .failed
        fps = 0
        guard shouldRun else { return }
        retryTask = Task { [weak self] in
            try? await Task.sleep(for: .seconds(2))
            guard !Task.isCancelled else { return }
            self?.retryTask = nil
            self?.update()
        }
    }
}

/// URLSession delegate. Runs on its own serial queue, so the buffer needs no lock.
private final class MJPEGReceiver: NSObject, URLSessionDataDelegate, @unchecked Sendable {
    private var buffer = Data()
    private let onFrame: @Sendable (UIImage, Data) -> Void
    private let onEnd: @Sendable () -> Void

    private static let soi = Data([0xFF, 0xD8])
    private static let eoi = Data([0xFF, 0xD9])
    private static let maxBuffer = 8 * 1024 * 1024

    init(onFrame: @escaping @Sendable (UIImage, Data) -> Void, onEnd: @escaping @Sendable () -> Void) {
        self.onFrame = onFrame
        self.onEnd = onEnd
    }

    func urlSession(_ session: URLSession, dataTask: URLSessionDataTask, didReceive data: Data) {
        buffer.append(data)

        // Keep only the newest complete JPEG; older ones are already stale.
        var latest: Data?
        while let start = buffer.range(of: Self.soi),
              let end = buffer.range(of: Self.eoi, options: [], in: start.upperBound..<buffer.endIndex) {
            latest = buffer.subdata(in: start.lowerBound..<end.upperBound)
            buffer = buffer.subdata(in: end.upperBound..<buffer.endIndex)
        }
        if buffer.count > Self.maxBuffer { buffer.removeAll(keepingCapacity: false) }

        if let jpeg = latest, let img = UIImage(data: jpeg) {
            // Decode off the main thread so drawing it is cheap.
            onFrame(img.preparingForDisplay() ?? img, jpeg)
        }
    }

    func urlSession(_ session: URLSession, task: URLSessionTask, didCompleteWithError error: Error?) {
        onEnd()
    }
}
