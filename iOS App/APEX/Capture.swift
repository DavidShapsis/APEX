import Foundation
import UIKit
import AVFoundation
import Photos
import Observation

// Snapshot and video capture of the live feed, saved to the phone's Photos library.
// Snapshots keep the exact JPEG the robot sent. Video is re-encoded to H.264 (.mp4)
// from the frames as they arrive, timed in real time.

/// Owns the capture state shown by the buttons on the feed.
@Observable
@MainActor
final class CaptureController {
    private(set) var isRecording = false
    private(set) var recordingStart: Date?
    private(set) var isSavingPhoto = false
    private(set) var isFinalizing = false

    /// Set by RobotStore to show a toast: (message, success).
    @ObservationIgnored var onMessage: (@MainActor (String, Bool) -> Void)?

    private let feed: MJPEGStream
    @ObservationIgnored private var recorder: VideoRecorder?

    init(feed: MJPEGStream) {
        self.feed = feed
        feed.frameHandler = { [weak self] image in
            self?.recorder?.append(image, at: CACurrentMediaTime())
        }
        feed.onStopped = { [weak self] in
            self?.feedStopped()
        }
    }

    // MARK: Snapshot

    func snapshot() {
        guard !isSavingPhoto, feed.state == .streaming, let jpeg = feed.latestJPEG else { return }
        isSavingPhoto = true
        Haptics.tap()
        Task {
            do {
                try await MediaSaver.savePhoto(jpeg: jpeg)
                Haptics.success()
                onMessage?("Photo saved to Photos", true)
            } catch {
                Haptics.error()
                onMessage?(error.localizedDescription, false)
            }
            isSavingPhoto = false
        }
    }

    // MARK: Video

    func toggleRecording() {
        if isRecording { stopRecording() } else { startRecording() }
    }

    private func startRecording() {
        guard !isFinalizing, feed.state == .streaming else { return }
        let formatter = DateFormatter()
        formatter.dateFormat = "yyyyMMdd-HHmmss"
        let url = FileManager.default.temporaryDirectory
            .appendingPathComponent("APEX-\(formatter.string(from: Date())).mp4")
        recorder = VideoRecorder(url: url)
        recordingStart = Date()
        isRecording = true
        feed.acquire()   // keep the stream running even if the Drive tab is left
        Haptics.tap()
    }

    func stopRecording() {
        guard isRecording, let rec = recorder else { return }
        isRecording = false
        recordingStart = nil
        recorder = nil
        isFinalizing = true
        Haptics.tap()
        let background = UIApplication.shared.beginBackgroundTask(withName: "apex-save-video")
        Task {
            defer { UIApplication.shared.endBackgroundTask(background) }
            if let url = await rec.finish() {
                do {
                    try await MediaSaver.saveVideo(at: url)
                    Haptics.success()
                    onMessage?("Video saved to Photos", true)
                } catch {
                    Haptics.error()
                    onMessage?(error.localizedDescription, false)
                }
                try? FileManager.default.removeItem(at: url)
            } else {
                Haptics.error()
                onMessage?("Nothing was recorded.", false)
            }
            isFinalizing = false
        }
        feed.release()
    }

    /// The stream shut down (app went to the background, or no address). Save what we have.
    private func feedStopped() {
        if isRecording { stopRecording() }
    }
}

// MARK: - Photos

enum MediaSaveError: LocalizedError {
    case denied

    var errorDescription: String? {
        "Photos access is off for APEX. Turn it on in Settings > Privacy & Security > Photos."
    }
}

enum MediaSaver {
    private static func authorize() async throws {
        var status = PHPhotoLibrary.authorizationStatus(for: .addOnly)
        if status == .notDetermined {
            status = await PHPhotoLibrary.requestAuthorization(for: .addOnly)
        }
        guard status == .authorized || status == .limited else { throw MediaSaveError.denied }
    }

    static func savePhoto(jpeg: Data) async throws {
        try await authorize()
        try await PHPhotoLibrary.shared().performChanges {
            let request = PHAssetCreationRequest.forAsset()
            request.addResource(with: .photo, data: jpeg, options: nil)
        }
    }

    static func saveVideo(at url: URL) async throws {
        try await authorize()
        try await PHPhotoLibrary.shared().performChanges {
            _ = PHAssetChangeRequest.creationRequestForAssetFromVideo(atFileURL: url)
        }
    }
}

// MARK: - Video writer

/// Writes frames to an H.264 .mp4 on a private serial queue. Frame size comes from the
/// first frame. If the queue falls behind, frames are dropped rather than piling up.
final class VideoRecorder: @unchecked Sendable {
    let url: URL

    private let queue = DispatchQueue(label: "apex.video-recorder", qos: .userInitiated)
    private let lock = NSLock()
    private var pending = 0

    // Touched only on `queue`.
    private var writer: AVAssetWriter?
    private var input: AVAssetWriterInput?
    private var adaptor: AVAssetWriterInputPixelBufferAdaptor?
    private var width = 0
    private var height = 0
    private var startTime: CFTimeInterval = 0
    private var lastPTS = CMTime.invalid
    private var frames = 0
    private var failed = false

    init(url: URL) {
        self.url = url
        try? FileManager.default.removeItem(at: url)
    }

    func append(_ image: UIImage, at time: CFTimeInterval) {
        lock.lock()
        if pending >= 3 {
            lock.unlock()
            return
        }
        pending += 1
        lock.unlock()

        queue.async { [self] in
            write(image, at: time)
            lock.lock()
            pending -= 1
            lock.unlock()
        }
    }

    /// Finishes the file. Returns its URL, or nil if nothing usable was written.
    func finish() async -> URL? {
        await withCheckedContinuation { (continuation: CheckedContinuation<URL?, Never>) in
            queue.async { [self] in
                guard let writer, let input, frames > 0, writer.status == .writing else {
                    writer?.cancelWriting()
                    try? FileManager.default.removeItem(at: url)
                    continuation.resume(returning: nil)
                    return
                }
                input.markAsFinished()
                writer.endSession(atSourceTime: CMTimeAdd(lastPTS, CMTime(value: 1, timescale: 30)))
                let outputURL = url
                writer.finishWriting {
                    continuation.resume(returning: writer.status == .completed ? outputURL : nil)
                }
            }
        }
    }

    // MARK: Private, on `queue`

    private func write(_ image: UIImage, at time: CFTimeInterval) {
        guard !failed, let cg = image.cgImage else { return }
        if writer == nil && !setup(width: cg.width, height: cg.height, at: time) {
            failed = true
            return
        }
        guard let input, let adaptor, input.isReadyForMoreMediaData else { return }
        let pts = CMTime(seconds: time - startTime, preferredTimescale: 600)
        if lastPTS.isValid && pts <= lastPTS { return }
        guard let buffer = makeBuffer(from: cg) else { return }
        if adaptor.append(buffer, withPresentationTime: pts) {
            lastPTS = pts
            frames += 1
        } else {
            failed = true
        }
    }

    private func setup(width w: Int, height h: Int, at time: CFTimeInterval) -> Bool {
        width = max(2, w & ~1)     // H.264 wants even dimensions
        height = max(2, h & ~1)
        guard let wr = try? AVAssetWriter(outputURL: url, fileType: .mp4) else { return false }
        let settings: [String: Any] = [
            AVVideoCodecKey: AVVideoCodecType.h264,
            AVVideoWidthKey: width,
            AVVideoHeightKey: height,
            AVVideoCompressionPropertiesKey: [AVVideoAverageBitRateKey: max(2_000_000, width * height * 4)]
        ]
        let inp = AVAssetWriterInput(mediaType: .video, outputSettings: settings)
        inp.expectsMediaDataInRealTime = true
        let attributes: [String: Any] = [
            kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA,
            kCVPixelBufferWidthKey as String: width,
            kCVPixelBufferHeightKey as String: height
        ]
        let ad = AVAssetWriterInputPixelBufferAdaptor(assetWriterInput: inp, sourcePixelBufferAttributes: attributes)
        guard wr.canAdd(inp) else { return false }
        wr.add(inp)
        guard wr.startWriting() else { return false }
        wr.startSession(atSourceTime: .zero)
        writer = wr
        input = inp
        adaptor = ad
        startTime = time
        return true
    }

    private func makeBuffer(from cg: CGImage) -> CVPixelBuffer? {
        var buffer: CVPixelBuffer?
        let attributes = [
            kCVPixelBufferCGImageCompatibilityKey: kCFBooleanTrue!,
            kCVPixelBufferCGBitmapContextCompatibilityKey: kCFBooleanTrue!
        ] as CFDictionary
        CVPixelBufferCreate(kCFAllocatorDefault, width, height, kCVPixelFormatType_32BGRA, attributes, &buffer)
        guard let buffer else { return nil }
        CVPixelBufferLockBaseAddress(buffer, [])
        defer { CVPixelBufferUnlockBaseAddress(buffer, []) }
        guard let context = CGContext(
            data: CVPixelBufferGetBaseAddress(buffer),
            width: width,
            height: height,
            bitsPerComponent: 8,
            bytesPerRow: CVPixelBufferGetBytesPerRow(buffer),
            space: CGColorSpaceCreateDeviceRGB(),
            bitmapInfo: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue
        ) else { return nil }
        context.draw(cg, in: CGRect(x: 0, y: 0, width: width, height: height))
        return buffer
    }
}
