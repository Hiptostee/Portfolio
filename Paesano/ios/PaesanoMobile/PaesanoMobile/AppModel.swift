import CoreGraphics
import Foundation
import ImageIO
import UniformTypeIdentifiers

struct PoseState: Codable {
    let x: Double
    let y: Double
    let theta: Double
}

struct PathPoint: Codable, Hashable {
    let x: Double
    let y: Double
}

struct BridgeState: Codable {
    let mode: String
    let localizationMode: Bool
    let launchRunning: Bool
    let isNavigating: Bool
    let isPaused: Bool
    let canPause: Bool
    let canResume: Bool
    let canCancel: Bool
    let mapAvailable: Bool
    let mapRevision: Int
    let telemetryRevision: Int
    let mapYaml: String
    let mapSavePrefix: String
    let pose: PoseState?
    let path: [PathPoint]
    let lastError: String?

    static let idle = BridgeState(
        mode: "idle",
        localizationMode: false,
        launchRunning: false,
        isNavigating: false,
        isPaused: false,
        canPause: false,
        canResume: false,
        canCancel: false,
        mapAvailable: false,
        mapRevision: 0,
        telemetryRevision: 0,
        mapYaml: "",
        mapSavePrefix: "",
        pose: nil,
        path: [],
        lastError: nil
    )
}

struct MapOrigin: Codable {
    let x: Double
    let y: Double
    let yaw: Double
}

struct OccupancyMap: Codable {
    let frameId: String
    let resolution: Double
    let width: Int
    let height: Int
    let origin: MapOrigin
    let data: [Int]
    let revision: Int

    func cgImage() -> CGImage? {
        guard width > 0, height > 0, data.count == width * height else {
            return nil
        }

        var rgba = [UInt8](repeating: 255, count: width * height * 4)
        for row in 0..<height {
            for column in 0..<width {
                let sourceIndex = (height - 1 - row) * width + column
                let shade = shadeForOccupancy(data[sourceIndex])
                let pixelOffset = (row * width + column) * 4
                rgba[pixelOffset + 0] = shade
                rgba[pixelOffset + 1] = shade
                rgba[pixelOffset + 2] = shade
                rgba[pixelOffset + 3] = 255
            }
        }

        let provider = CGDataProvider(data: Data(rgba) as CFData)
        let colorSpace = CGColorSpaceCreateDeviceRGB()
        return CGImage(
            width: width,
            height: height,
            bitsPerComponent: 8,
            bitsPerPixel: 32,
            bytesPerRow: width * 4,
            space: colorSpace,
            bitmapInfo: CGBitmapInfo(rawValue: CGImageAlphaInfo.last.rawValue),
            provider: provider!,
            decode: nil,
            shouldInterpolate: false,
            intent: .defaultIntent
        )
    }

    func screenPoint(for worldPoint: CGPoint, in size: CGSize) -> CGPoint {
        let mapWidthMeters = Double(width) * resolution
        let mapHeightMeters = Double(height) * resolution
        guard mapWidthMeters > 0, mapHeightMeters > 0 else {
            return .zero
        }

        let normalizedX = (Double(worldPoint.x) - origin.x) / mapWidthMeters
        let normalizedY = (Double(worldPoint.y) - origin.y) / mapHeightMeters

        return CGPoint(
            x: normalizedX * size.width,
            y: (1.0 - normalizedY) * size.height
        )
    }

    func worldPoint(for screenPoint: CGPoint, in size: CGSize) -> CGPoint {
        guard size.width > 0, size.height > 0 else {
            return .zero
        }

        let normalizedX = screenPoint.x / size.width
        let normalizedY = 1.0 - (screenPoint.y / size.height)
        return CGPoint(
            x: origin.x + Double(normalizedX) * Double(width) * resolution,
            y: origin.y + Double(normalizedY) * Double(height) * resolution
        )
    }
}

private struct StateEnvelope: Codable {
    let type: String
    let state: BridgeState?
    let map: OccupancyMap?
    let message: String?
}

private struct HealthResponse: Codable {
    let status: String
    let state: BridgeState
}

private struct ErrorResponse: Codable {
    let detail: String
}

private struct MapSaveResponse: Codable {
    let mapYaml: String
}

private struct GoalResponse: Codable {
    let success: Bool
    let pathPoints: Int
}

private struct ModeRequest: Encodable {
    let mode: String
}

private struct GoalRequest: Encodable {
    let xM: Double
    let yM: Double
}

private struct EmptyBody: Codable {}

private struct AnyEncodable: Encodable {
    private let encodeClosure: (Encoder) throws -> Void

    init(_ wrapped: Encodable) {
        self.encodeClosure = { encoder in
            try wrapped.encode(to: encoder)
        }
    }

    func encode(to encoder: Encoder) throws {
        try encodeClosure(encoder)
    }
}

@MainActor
final class AppModel: ObservableObject {
    private static let savedAddressKey = "PaesanoSavedAddress"

    @Published var manualAddress: String
    @Published var connectionStatus = "Enter a robot IP or Tailscale address."
    @Published var inlineError: String?
    @Published var isConnected = false
    @Published var bridgeState = BridgeState.idle
    @Published var occupancyMap: OccupancyMap?
    @Published var mapImage: CGImage?
    @Published var isSwitchingMode = false
    @Published var isSavingMap = false
    @Published var toastMessage: String?

    private let decoder: JSONDecoder
    private let encoder: JSONEncoder
    private let session = URLSession(configuration: .default)

    private var baseURL: URL?
    private var webSocketTask: URLSessionWebSocketTask?
    private var receiveTask: Task<Void, Never>?
    private var statePollingTask: Task<Void, Never>?
    private var webSocketReconnectTask: Task<Void, Never>?
    private var teleopTask: Task<Void, Never>?
    private var teleopTaskGeneration = 0
    private var translationVector = CGPoint.zero
    private var rotationVector = 0.0
    private var isConnecting = false

    init() {
        let decoder = JSONDecoder()
        decoder.keyDecodingStrategy = .convertFromSnakeCase
        self.decoder = decoder

        let encoder = JSONEncoder()
        encoder.keyEncodingStrategy = .convertToSnakeCase
        self.encoder = encoder

        self.manualAddress = UserDefaults.standard.string(forKey: Self.savedAddressKey) ?? ""

        if !manualAddress.isEmpty {
            connectionStatus = "Trying \(manualAddress)…"
            Task {
                await connectManualAddress()
            }
        }
    }

    func saveCurrentAddress() {
        let trimmed = manualAddress.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else {
            inlineError = "Enter an address before saving."
            return
        }
        UserDefaults.standard.set(trimmed, forKey: Self.savedAddressKey)
        connectionStatus = "Saved \(trimmed)"
    }

    func loadSavedAddress() {
        manualAddress = UserDefaults.standard.string(forKey: Self.savedAddressKey) ?? ""
        if manualAddress.isEmpty {
            connectionStatus = "No saved address found."
        } else {
            connectionStatus = "Loaded \(manualAddress)"
        }
    }

    func connectManualAddress() async {
        let trimmed = manualAddress.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmed.isEmpty else {
            inlineError = "Enter a robot IP or hostname."
            return
        }

        let cleaned = trimmed.replacingOccurrences(of: "http://", with: "")
        let parts = cleaned.split(separator: ":")
        let host = String(parts.first ?? "")
        let port = parts.count > 1 ? Int(parts[1]) ?? 8000 : 8000
        await connect(host: host, port: port)
    }

    func setMode(_ mode: String) async {
        isSwitchingMode = true
        inlineError = nil
        do {
            let state: BridgeState = try await performRequest(
                path: "/mode",
                method: "POST",
                body: ModeRequest(mode: mode)
            )
            apply(state: state)
            await fetchMapIfNeeded(expectedRevision: state.mapRevision, force: true)
            showToast("Switched to \(mode.capitalized)")
        } catch {
            inlineError = error.localizedDescription
        }
        isSwitchingMode = false
    }

    func saveMap() async {
        isSavingMap = true
        inlineError = nil
        do {
            let response: MapSaveResponse = try await performRequest(
                path: "/mapping/save",
                method: "POST",
                body: EmptyBody()
            )
            connectionStatus = "Saved map to \(response.mapYaml)"
            showToast("Map saved as \((response.mapYaml as NSString).lastPathComponent)")
        } catch {
            inlineError = error.localizedDescription
        }
        isSavingMap = false
    }

    func sendGoal(worldPoint: CGPoint) async {
        do {
            let _: GoalResponse = try await performRequest(
                path: "/navigation/goal",
                method: "POST",
                body: GoalRequest(xM: worldPoint.x, yM: worldPoint.y)
            )
            inlineError = nil
            await refreshState()
        } catch {
            inlineError = error.localizedDescription
        }
    }

    func pauseNavigation() async {
        do {
            let state: BridgeState = try await performRequest(
                path: "/navigation/pause",
                method: "POST",
                body: EmptyBody()
            )
            apply(state: state)
            try? await Task.sleep(for: .milliseconds(150))
            await refreshState()
        } catch {
            inlineError = error.localizedDescription
        }
    }

    func resumeNavigation() async {
        do {
            let state: BridgeState = try await performRequest(
                path: "/navigation/resume",
                method: "POST",
                body: EmptyBody()
            )
            apply(state: state)
            try? await Task.sleep(for: .milliseconds(150))
            await refreshState()
        } catch {
            inlineError = error.localizedDescription
        }
    }

    func togglePauseResume() async {
        if bridgeState.isPaused {
            await resumeNavigation()
        } else {
            await pauseNavigation()
        }
    }

    func cancelNavigation() async {
        do {
            let state: BridgeState = try await performRequest(
                path: "/navigation/cancel",
                method: "POST",
                body: EmptyBody()
            )
            apply(state: state)
        } catch {
            inlineError = error.localizedDescription
        }
    }

    func stopRobot() async {
        do {
            let state: BridgeState = try await performRequest(
                path: "/navigation/stop",
                method: "POST",
                body: EmptyBody()
            )
            apply(state: state)
        } catch {
            inlineError = error.localizedDescription
        }
    }

    func beginTeleop() {
        if let teleopTask, teleopTask.isCancelled {
            self.teleopTask = nil
        }
        guard teleopTask == nil else {
            return
        }

        teleopTaskGeneration += 1
        let generation = teleopTaskGeneration
        teleopTask = Task { [weak self] in
            defer {
                Task { @MainActor [weak self] in
                    guard let self, self.teleopTaskGeneration == generation else {
                        return
                    }
                    self.teleopTask = nil
                }
            }

            while let self, !Task.isCancelled {
                await self.sendTeleop(deadman: true)
                try? await Task.sleep(for: .milliseconds(80))
            }
        }
    }

    func updateTeleop(translation: CGPoint, rotation: Double) {
        translationVector = translation
        rotationVector = rotation
    }

    func endTeleop() {
        teleopTaskGeneration += 1
        teleopTask?.cancel()
        teleopTask = nil
        translationVector = .zero
        rotationVector = 0.0
        Task {
            await sendTeleop(deadman: false)
        }
    }

    private func connect(host: String, port: Int) async {
        guard !isConnecting else {
            return
        }
        guard let httpURL = URL(string: "http://\(host):\(port)") else {
            inlineError = "Invalid address."
            return
        }

        isConnecting = true
        defer { isConnecting = false }

        inlineError = nil
        connectionStatus = "Connecting to \(host):\(port)…"
        baseURL = httpURL

        do {
            let response: HealthResponse = try await performRequest(path: "/health")
            apply(state: response.state)
            await fetchMapIfNeeded(expectedRevision: response.state.mapRevision, force: true)
            connectionStatus = "Connected to \(host):\(port)"
            isConnected = true
            manualAddress = "\(host):\(port)"
            UserDefaults.standard.set(manualAddress, forKey: Self.savedAddressKey)
            startStatePolling()
            openWebSocket(host: host, port: port)
        } catch {
            isConnected = false
            baseURL = nil
            stopStatePolling()
            inlineError = error.localizedDescription
            connectionStatus = "Connection failed."
        }
    }

    private func openWebSocket(host: String, port: Int) {
        webSocketTask?.cancel(with: .goingAway, reason: nil)
        receiveTask?.cancel()
        webSocketReconnectTask?.cancel()
        webSocketReconnectTask = nil

        guard let websocketURL = URL(string: "ws://\(host):\(port)/ws") else {
            inlineError = "Invalid WebSocket address."
            return
        }

        let task = session.webSocketTask(with: websocketURL)
        webSocketTask = task
        task.resume()
        receiveTask = Task { [weak self] in
            await self?.receiveLoop(task: task)
        }
    }

    private func receiveLoop(task: URLSessionWebSocketTask) async {
        while !Task.isCancelled {
            do {
                let message = try await task.receive()
                switch message {
                case let .string(text):
                    try await handleWebSocketText(text)
                case let .data(data):
                    if let text = String(data: data, encoding: .utf8) {
                        try await handleWebSocketText(text)
                    }
                @unknown default:
                    break
                }
            } catch {
                webSocketTask = nil
                receiveTask = nil
                connectionStatus = "Connected. Realtime link lost; retrying…"
                endTeleop()
                scheduleWebSocketReconnect()
                break
            }
        }
    }

    private func handleWebSocketText(_ text: String) async throws {
        let envelope = try decoder.decode(StateEnvelope.self, from: Data(text.utf8))
        switch envelope.type {
        case "state":
            if let state = envelope.state {
                apply(state: state)
                await fetchMapIfNeeded(expectedRevision: state.mapRevision)
            }
        case "map":
            if let map = envelope.map {
                occupancyMap = map
                mapImage = map.cgImage()
            }
        case "error":
            inlineError = envelope.message ?? "Bridge error."
        default:
            break
        }
    }

    private func apply(state: BridgeState) {
        let wasNavigating = bridgeState.isNavigating
        
        // only end teleop when TRANSITIONING to navigating, not every update
        if !wasNavigating && state.isNavigating && state.mode == "localization" {
            endTeleop()
        }
        
        if wasNavigating && !state.isNavigating {
            inlineError = nil
        }
        
        bridgeState = state
        if !state.mapAvailable {
            occupancyMap = nil
            mapImage = nil
        }
    }

    private func showToast(_ message: String) {
        toastMessage = message
        Task { [weak self] in
            try? await Task.sleep(for: .seconds(2))
            guard let self, self.toastMessage == message else {
                return
            }
            self.toastMessage = nil
        }
    }

    private func startStatePolling() {
        statePollingTask?.cancel()
        statePollingTask = Task { [weak self] in
            while let self, !Task.isCancelled {
                await self.refreshState()
                try? await Task.sleep(for: .milliseconds(350))
            }
        }
    }

    private func stopStatePolling() {
        statePollingTask?.cancel()
        statePollingTask = nil
        webSocketReconnectTask?.cancel()
        webSocketReconnectTask = nil
    }

    private func scheduleWebSocketReconnect() {
        guard webSocketReconnectTask == nil,
              isConnected,
              let baseURL,
              let host = baseURL.host
        else {
            return
        }

        let port = baseURL.port ?? (baseURL.scheme == "https" ? 443 : 80)
        webSocketReconnectTask = Task { [weak self] in
            try? await Task.sleep(for: .seconds(1))
            guard let self, !Task.isCancelled, self.isConnected else {
                return
            }
            self.openWebSocket(host: host, port: port)
        }
    }

    private func refreshState() async {
        guard baseURL != nil else {
            return
        }

        do {
            let state: BridgeState = try await performRequest(path: "/state")
            apply(state: state)
            // remove the inlineError = nil line here
            await fetchMapIfNeeded(expectedRevision: state.mapRevision)
        } catch {
            if webSocketTask == nil {
                isConnected = false
                baseURL = nil
                stopStatePolling()
                connectionStatus = "Disconnected."
                inlineError = error.localizedDescription
            }
        }
    }

    private func fetchMapIfNeeded(expectedRevision: Int, force: Bool = false) async {
        guard bridgeState.mapAvailable else {
            return
        }
        guard force || occupancyMap?.revision != expectedRevision else {
            return
        }

        do {
            let map: OccupancyMap = try await performRequest(path: "/map/current")
            occupancyMap = map
            mapImage = map.cgImage()
        } catch {
            inlineError = error.localizedDescription
        }
    }

    private func sendTeleop(deadman: Bool) async {
        let linearX = Double(-translationVector.y)
        let linearY = Double(-translationVector.x)
        let angularZ = -rotationVector
        print(
            "sendTeleop: deadman=\(deadman), webSocketTask=\(webSocketTask == nil ? "nil - BLOCKED" : "exists"), linear_x=\(linearX), linear_y=\(linearY), angular_z=\(angularZ)"
        )
        guard let webSocketTask else {
            return
        }

        let payload: [String: Any] = [
            "type": "teleop",
            "deadman": deadman,
            "linear_x": linearX,
            "linear_y": linearY,
            "angular_z": angularZ,
        ]

        do {
            let data = try JSONSerialization.data(withJSONObject: payload)
            let text = String(decoding: data, as: UTF8.self)
            try await webSocketTask.send(.string(text))
        } catch {
            inlineError = error.localizedDescription
        }
    }

    private func performRequest<Response: Decodable>(
        path: String,
        method: String = "GET",
        body: Encodable? = nil
    ) async throws -> Response {
        guard let baseURL else {
            throw URLError(.badURL)
        }

        let normalizedPath = path.trimmingCharacters(in: CharacterSet(charactersIn: "/"))
        var request = URLRequest(url: baseURL.appendingPathComponent(normalizedPath))
        request.httpMethod = method
        request.timeoutInterval = 10.0

        if let body {
            request.httpBody = try encoder.encode(AnyEncodable(body))
            request.setValue("application/json", forHTTPHeaderField: "Content-Type")
        }

        let (data, response) = try await session.data(for: request)
        guard let httpResponse = response as? HTTPURLResponse else {
            throw URLError(.badServerResponse)
        }

        guard (200..<300).contains(httpResponse.statusCode) else {
            if let errorResponse = try? decoder.decode(ErrorResponse.self, from: data) {
                throw NSError(
                    domain: "PaesanoMobile",
                    code: httpResponse.statusCode,
                    userInfo: [NSLocalizedDescriptionKey: errorResponse.detail]
                )
            }
            throw NSError(
                domain: "PaesanoMobile",
                code: httpResponse.statusCode,
                userInfo: [NSLocalizedDescriptionKey: "Request to \(path) failed with status \(httpResponse.statusCode)."]
            )
        }

        return try decoder.decode(Response.self, from: data)
    }
}

private func shadeForOccupancy(_ value: Int) -> UInt8 {
    switch value {
    case ..<0:
        return 150
    case 0...100:
        return UInt8(max(0, 255 - (value * 2)))
    default:
        return 0
    }
}
