import SwiftUI

struct ContentView: View {
    @StateObject private var model = AppModel()

    var body: some View {
        ZStack {
            LinearGradient(
                colors: [Color(red: 0.08, green: 0.11, blue: 0.12), Color(red: 0.18, green: 0.21, blue: 0.19)],
                startPoint: .topLeading,
                endPoint: .bottomTrailing
            )
            .ignoresSafeArea()

            if model.isConnected {
                dashboard
            } else {
                ConnectionView(model: model)
            }

            if model.isSwitchingMode {
                loadingOverlay
            }

            if let toast = model.toastMessage {
                toastOverlay(message: toast)
            }
        }
    }

    @ViewBuilder
    private var dashboard: some View {
        switch model.bridgeState.mode {
        case "mapping":
            MappingView(model: model)
        case "localization":
            NavigationView(model: model)
        default:
            IdleView(model: model)
        }
    }

    private var loadingOverlay: some View {
        ZStack {
            Color.black.opacity(0.28)
                .ignoresSafeArea()
            VStack(spacing: 14) {
                ProgressView()
                    .controlSize(.large)
                    .tint(.white)
                Text("Switching modes…")
                    .font(.headline)
                    .foregroundStyle(.white)
            }
            .padding(24)
            .background(Color.black.opacity(0.72), in: RoundedRectangle(cornerRadius: 22, style: .continuous))
        }
    }

    private func toastOverlay(message: String) -> some View {
        VStack {
            Spacer()
            Text(message)
                .font(.headline)
                .foregroundStyle(.white)
                .padding(.horizontal, 18)
                .padding(.vertical, 14)
                .background(Color.black.opacity(0.78), in: Capsule())
                .padding(.bottom, 28)
        }
        .transition(.move(edge: .bottom).combined(with: .opacity))
    }
}

private struct ConnectionView: View {
    @ObservedObject var model: AppModel

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 24) {
                Text("Paesano Mobile")
                    .font(.system(size: 40, weight: .bold, design: .rounded))
                    .foregroundStyle(.white)

                Text("Campus Wi-Fi is unreliable for device discovery. Enter the Pi IP or Tailscale address directly.")
                    .font(.headline)
                    .foregroundStyle(Color.white.opacity(0.72))

                VStack(alignment: .leading, spacing: 12) {
                    Text("Robot Address")
                        .font(.title3.weight(.semibold))
                        .foregroundStyle(.white)

                    TextField("100.x.x.x:8000 or 10.x.x.x:8000", text: $model.manualAddress)
                        .textInputAutocapitalization(.never)
                        .autocorrectionDisabled()
                        .padding()
                        .background(Color.white.opacity(0.92), in: RoundedRectangle(cornerRadius: 16, style: .continuous))

                    HStack(spacing: 12) {
                        Button("Connect") {
                            Task { await model.connectManualAddress() }
                        }
                        .buttonStyle(ActionButtonStyle(tint: Color(red: 0.81, green: 0.63, blue: 0.27)))

                        Button("Save") {
                            model.saveCurrentAddress()
                        }
                        .buttonStyle(ActionButtonStyle(tint: Color(red: 0.18, green: 0.57, blue: 0.52)))

                        Button("Load") {
                            model.loadSavedAddress()
                        }
                        .buttonStyle(ActionButtonStyle(tint: Color(red: 0.31, green: 0.48, blue: 0.80)))
                    }
                }

                statusCard
            }
            .padding(24)
            .frame(maxWidth: 720)
        }
    }

    private var statusCard: some View {
        VStack(alignment: .leading, spacing: 8) {
            Text("Status")
                .font(.title3.weight(.semibold))
                .foregroundStyle(.white)

            Text(model.inlineError ?? model.connectionStatus)
                .foregroundStyle(Color.black.opacity(0.82))
                .frame(maxWidth: .infinity, alignment: .leading)
                .padding()
                .background(Color.white.opacity(0.92), in: RoundedRectangle(cornerRadius: 18, style: .continuous))
        }
    }
}

private struct IdleView: View {
    @ObservedObject var model: AppModel

    var body: some View {
        ScrollView {
            VStack(spacing: 22) {
                dashboardHeader(
                    title: "Bridge Online",
                    subtitle: "Choose a mode. The bridge will restart the single ROS launch file with localization enabled or disabled.",
                    model: model
                )

                MapPanel(model: model, tapAction: nil)
                    .frame(maxHeight: 420)
            }
            .padding(24)
        }
    }
}

private struct MappingView: View {
    @ObservedObject var model: AppModel
    @State private var translation = CGPoint.zero
    @State private var rotation = 0.0
    @State private var translationActive = false
    @State private var rotationActive = false
    @State private var confirmSaveMap = false

    var body: some View {
        ScrollView {
            VStack(spacing: 18) {
                dashboardHeader(
                    title: "Mapping Mode",
                    subtitle: "Drive with the joystick, build the map live, and save over the canonical hardware map when coverage looks good.",
                    model: model
                )
                MapPanel(model: model, tapAction: nil)
                HStack(spacing: 12) {
                    Button("Save Map") {
                        confirmSaveMap = true
                    }
                    .buttonStyle(ActionButtonStyle(tint: Color(red: 0.81, green: 0.63, blue: 0.27)))
                    .disabled(model.isSavingMap)
                }

                HStack(spacing: 24) {
                    driveJoystick
                    turnJoystick
                }
            }
            .padding(24)
        }
        .confirmationDialog("Overwrite the active hardware map?", isPresented: $confirmSaveMap, titleVisibility: .visible) {
            Button("Save Map", role: .destructive) {
                Task { await model.saveMap() }
            }
            Button("Cancel", role: .cancel) {}
        } message: {
            Text("This will replace the current canonical map file used for localization.")
        }
    }

    private var driveJoystick: some View {
        JoystickPad(title: "Translate", tint: Color(red: 0.81, green: 0.63, blue: 0.27), axisMode: .planar) { vector in
            translationActive = true
            translation = vector
            pushTeleop()
        } onEnded: {
            translationActive = false
            translation = .zero
            finishTeleop()
        }
    }

    private var turnJoystick: some View {
        JoystickPad(title: "Rotate", tint: Color(red: 0.18, green: 0.57, blue: 0.52), axisMode: .horizontal) { vector in
            rotationActive = true
            rotation = Double(vector.x)
            pushTeleop()
        } onEnded: {
            rotationActive = false
            rotation = 0.0
            finishTeleop()
        }
    }

    private func pushTeleop() {
        model.updateTeleop(translation: translation, rotation: rotation)
        model.beginTeleop()
    }

    private func finishTeleop() {
        model.updateTeleop(translation: translation, rotation: rotation)
        if translationActive || rotationActive {
            model.beginTeleop()
        } else {
            model.endTeleop()
        }
    }
}

private struct NavigationView: View {
    @ObservedObject var model: AppModel
    @State private var translation = CGPoint.zero
    @State private var rotation = 0.0
    @State private var translationActive = false
    @State private var rotationActive = false

    private var joystickEnabled: Bool {
        !model.bridgeState.isNavigating
    }

    private var pauseResumeTitle: String {
        model.bridgeState.isPaused ? "Resume" : "Pause"
    }

    private var canTogglePauseResume: Bool {
        model.bridgeState.canPause || model.bridgeState.canResume
    }

    var body: some View {
        ScrollView {
            VStack(spacing: 18) {
                dashboardHeader(
                    title: "Navigation Mode",
                    subtitle: "Tap the map to send an A* goal. Teleop is available whenever path following is idle or paused.",
                    model: model
                )
                MapPanel(model: model) { worldPoint in
                    Task { await model.sendGoal(worldPoint: worldPoint) }
                }

                HStack(spacing: 12) {
                    Button(pauseResumeTitle) {
                        Task { await model.togglePauseResume() }
                    }
                    .buttonStyle(ActionButtonStyle(tint: model.bridgeState.isPaused ? Color(red: 0.18, green: 0.57, blue: 0.52) : Color(red: 0.78, green: 0.50, blue: 0.24)))
                    .disabled(!canTogglePauseResume)

                    Button("Cancel Path") {
                        Task { await model.cancelNavigation() }
                    }
                    .buttonStyle(ActionButtonStyle(tint: Color(red: 0.78, green: 0.31, blue: 0.25)))
                    .disabled(!model.bridgeState.canCancel)
                }

                HStack(spacing: 24) {
                    localizedDriveJoystick
                    localizedTurnJoystick
                }
            }
            .padding(24)
        }
    }

    private var localizedDriveJoystick: some View {
        JoystickPad(title: "Translate", tint: Color(red: 0.81, green: 0.63, blue: 0.27), axisMode: .planar) { vector in
            translationActive = true
            translation = vector
            if joystickEnabled {
                pushTeleop()
            }
        } onEnded: {
            translationActive = false
            translation = .zero
            finishTeleop()
        }
        .opacity(joystickEnabled ? 1.0 : 0.45)
    }

    private var localizedTurnJoystick: some View {
        JoystickPad(title: "Rotate", tint: Color(red: 0.18, green: 0.57, blue: 0.52), axisMode: .horizontal) { vector in
            rotationActive = true
            rotation = Double(vector.x)
            if joystickEnabled {
                pushTeleop()
            }
        } onEnded: {
            rotationActive = false
            rotation = 0.0
            finishTeleop()
        }
        .opacity(joystickEnabled ? 1.0 : 0.45)
    }

    private func pushTeleop() {
        guard joystickEnabled else { return }
        model.updateTeleop(translation: translation, rotation: rotation)
        model.beginTeleop()
    }

    private func finishTeleop() {
        model.updateTeleop(translation: translation, rotation: rotation)
        if joystickEnabled && (translationActive || rotationActive) {
            model.beginTeleop()
        } else {
            model.endTeleop()
        }
    }
}

private struct MapPanel: View {
    @ObservedObject var model: AppModel
    let tapAction: ((CGPoint) -> Void)?

    private var rawMapAspectRatio: CGFloat {
        guard let map = model.occupancyMap, map.height > 0 else {
            return 1.0
        }
        return CGFloat(map.width) / CGFloat(map.height)
    }

    private var shouldRotateMap: Bool {
        rawMapAspectRatio < 0.9
    }

    private var displayAspectRatio: CGFloat {
        shouldRotateMap ? (1.0 / max(rawMapAspectRatio, 0.01)) : rawMapAspectRatio
    }

    init(model: AppModel, tapAction: ((CGPoint) -> Void)? = nil) {
        self.model = model
        self.tapAction = tapAction
    }

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            if let error = model.inlineError ?? model.bridgeState.lastError {
                Text(error)
                    .font(.subheadline)
                    .foregroundStyle(Color(red: 1.0, green: 0.78, blue: 0.77))
            }

            MapCanvasView(
                map: model.occupancyMap,
                mapImage: model.mapImage,
                pose: model.bridgeState.pose,
                path: model.bridgeState.path,
                rotateClockwise: shouldRotateMap,
                tapAction: tapAction
            )
            .aspectRatio(displayAspectRatio, contentMode: .fit)
            .frame(maxWidth: 680, minHeight: 280, maxHeight: 420)
            .frame(maxWidth: .infinity, alignment: .center)
        }
        .padding()
        .background(Color.white.opacity(0.08), in: RoundedRectangle(cornerRadius: 28, style: .continuous))
    }
}

private struct MapCanvasView: View {
    let map: OccupancyMap?
    let mapImage: CGImage?
    let pose: PoseState?
    let path: [PathPoint]
    let rotateClockwise: Bool
    let tapAction: ((CGPoint) -> Void)?

    @State private var scale: CGFloat = 1.0
    @State private var lastScale: CGFloat = 1.0
    
    @State private var offset: CGSize = .zero
    @State private var lastOffset: CGSize = .zero

    var body: some View {
        GeometryReader { geometry in
            ZStack {
                RoundedRectangle(cornerRadius: 22, style: .continuous)
                    .fill(Color.black.opacity(0.28))

                if let map, let mapImage {
                    Canvas { context, size in
                        drawMapImage(context: context, mapImage: mapImage, size: size)
                        drawGrid(context: context, size: size)
                        drawPath(context: context, map: map, size: size)
                        drawRobot(context: context, map: map, size: size)
                    }
                    // Apply both scale AND offset
                    .scaleEffect(scale)
                    .offset(offset)
                }
            }
            .contentShape(Rectangle())
            .gesture(
                DragGesture()
                    .onChanged { value in
                        let proposedOffset = CGSize(
                            width: lastOffset.width + value.translation.width,
                            height: lastOffset.height + value.translation.height
                        )
                        offset = clampedOffset(
                            proposedOffset,
                            in: geometry.size,
                            scale: scale
                        )
                    }
                    .onEnded { _ in
                        offset = clampedOffset(offset, in: geometry.size, scale: scale)
                        lastOffset = offset
                    },
                including: scale > 1.01 ? .all : .none
            )
            .simultaneousGesture(
                MagnifyGesture()
                    .onChanged { value in
                        scale = clampedScale(lastScale * value.magnification)
                        offset = clampedOffset(lastOffset, in: geometry.size, scale: scale)
                    }
                    .onEnded { _ in
                        scale = clampedScale(scale)
                        offset = clampedOffset(offset, in: geometry.size, scale: scale)
                        lastScale = scale
                        lastOffset = offset
                    }
            )
            .simultaneousGesture(
                SpatialTapGesture()
                    .onEnded { event in
                        guard let map, let tapAction else { return }
                        let adjustedPoint = CGPoint(
                            x: (event.location.x - offset.width - (geometry.size.width/2)) / scale + (geometry.size.width/2),
                            y: (event.location.y - offset.height - (geometry.size.height/2)) / scale + (geometry.size.height/2)
                        )
                        tapAction(displayWorldPoint(for: adjustedPoint, map: map, in: geometry.size))
                    }
            )
        }
        .clipped() // Prevents the zoomed map from bleeding over other UI elements
    }

    private func drawMapImage(context: GraphicsContext, mapImage: CGImage, size: CGSize) {
        if rotateClockwise {
            var rotatedContext = context
            rotatedContext.translateBy(x: size.width, y: 0)
            rotatedContext.rotate(by: .degrees(90))
            rotatedContext.draw(
                Image(decorative: mapImage, scale: 1.0, orientation: .up),
                in: CGRect(origin: .zero, size: CGSize(width: size.height, height: size.width))
            )
        } else {
            context.draw(
                Image(decorative: mapImage, scale: 1.0, orientation: .up),
                in: CGRect(origin: .zero, size: size)
            )
        }
    }

    private func drawGrid(context: GraphicsContext, size: CGSize) {
        let spacing = max(size.width, size.height) / 10.0
        var gridPath = Path()

        stride(from: 0.0, through: size.width, by: spacing).forEach { x in
            gridPath.move(to: CGPoint(x: x, y: 0))
            gridPath.addLine(to: CGPoint(x: x, y: size.height))
        }
        stride(from: 0.0, through: size.height, by: spacing).forEach { y in
            gridPath.move(to: CGPoint(x: 0, y: y))
            gridPath.addLine(to: CGPoint(x: size.width, y: y))
        }

        context.stroke(gridPath, with: .color(Color.white.opacity(0.08)), lineWidth: 1)
    }

    private func drawPath(context: GraphicsContext, map: OccupancyMap, size: CGSize) {
        guard path.count > 1 else {
            return
        }

        var route = Path()
        route.move(to: displayPoint(for: CGPoint(x: path[0].x, y: path[0].y), map: map, in: size))
        for point in path.dropFirst() {
            route.addLine(to: displayPoint(for: CGPoint(x: point.x, y: point.y), map: map, in: size))
        }

        context.stroke(route, with: .color(Color(red: 0.34, green: 0.94, blue: 0.45)), lineWidth: 3)
    }

    private func drawRobot(context: GraphicsContext, map: OccupancyMap, size: CGSize) {
        guard let pose else {
            return
        }

        var mutableContext = context
        let center = displayPoint(for: CGPoint(x: pose.x, y: pose.y), map: map, in: size)
        let headingOffset = (.pi / 2.0) + (rotateClockwise ? (.pi / 2.0) : 0.0)
        let heading = CGFloat(-pose.theta + headingOffset)
        let radius: CGFloat = 11

        var arrow = Path()
        arrow.move(to: CGPoint(x: 0, y: -radius))
        arrow.addLine(to: CGPoint(x: radius * 0.8, y: radius))
        arrow.addLine(to: CGPoint(x: 0, y: radius * 0.3))
        arrow.addLine(to: CGPoint(x: -radius * 0.8, y: radius))
        arrow.closeSubpath()

        mutableContext.translateBy(x: center.x, y: center.y)
        mutableContext.rotate(by: .radians(Double(heading)))
        mutableContext.fill(arrow, with: .color(Color(red: 1.0, green: 0.38, blue: 0.32)))
    }

    private func clampedScale(_ scale: CGFloat) -> CGFloat {
        min(max(scale, 1.0), 4.0)
    }

    private func displayPoint(for worldPoint: CGPoint, map: OccupancyMap, in size: CGSize) -> CGPoint {
        if !rotateClockwise {
            return map.screenPoint(for: worldPoint, in: size)
        }

        let baseSize = CGSize(width: size.height, height: size.width)
        let unrotatedPoint = map.screenPoint(for: worldPoint, in: baseSize)
        return CGPoint(
            x: size.width - unrotatedPoint.y,
            y: unrotatedPoint.x
        )
    }

    private func displayWorldPoint(for displayPoint: CGPoint, map: OccupancyMap, in size: CGSize) -> CGPoint {
        if !rotateClockwise {
            return map.worldPoint(for: displayPoint, in: size)
        }

        let baseSize = CGSize(width: size.height, height: size.width)
        let unrotatedPoint = CGPoint(
            x: displayPoint.y,
            y: size.width - displayPoint.x
        )
        return map.worldPoint(for: unrotatedPoint, in: baseSize)
    }

    private func clampedOffset(_ proposedOffset: CGSize, in size: CGSize, scale: CGFloat) -> CGSize {
        guard scale > 1.0 else {
            return .zero
        }

        let maxX = ((size.width * scale) - size.width) / 2.0
        let maxY = ((size.height * scale) - size.height) / 2.0

        return CGSize(
            width: min(max(proposedOffset.width, -maxX), maxX),
            height: min(max(proposedOffset.height, -maxY), maxY)
        )
    }
}

private enum JoystickAxisMode {
    case planar
    case horizontal
}

private struct JoystickPad: View {
    let title: String
    let tint: Color
    let axisMode: JoystickAxisMode
    let onChanged: (CGPoint) -> Void
    let onEnded: () -> Void

    @State private var knobOffset = CGSize.zero
    private let radius: CGFloat = 82

    var body: some View {
        VStack(spacing: 12) {
            Text(title)
                .font(.headline)
                .foregroundStyle(.white)
                .frame(maxWidth: .infinity, alignment: .leading)
                .padding(.leading, 6)
            ZStack {
                Circle()
                    .fill(Color.white.opacity(0.08))
                    .frame(width: radius * 2, height: radius * 2)
                Circle()
                    .stroke(Color.white.opacity(0.22), lineWidth: 2)
                    .frame(width: radius * 1.3, height: radius * 1.3)
                Circle()
                    .fill(tint)
                    .frame(width: 72, height: 72)
                    .offset(knobOffset)
            }
        }
        .gesture(
            DragGesture(minimumDistance: 0)
                .onChanged { value in
                    let bounded = boundedOffset(for: value.translation)
                    knobOffset = bounded
                    onChanged(
                        CGPoint(
                            x: bounded.width / radius,
                            y: bounded.height / radius
                        )
                    )
                }
                .onEnded { _ in
                    withAnimation(.spring(response: 0.24, dampingFraction: 0.7)) {
                        knobOffset = .zero
                    }
                    onEnded()
                }
        )
    }

    private func boundedOffset(for translation: CGSize) -> CGSize {
        switch axisMode {
        case .horizontal:
            return CGSize(width: min(max(translation.width, -radius), radius), height: 0)
        case .planar:
            let length = sqrt((translation.width * translation.width) + (translation.height * translation.height))
            guard length > radius else {
                return translation
            }
            let scale = radius / max(length, 1)
            return CGSize(width: translation.width * scale, height: translation.height * scale)
        }
    }
}

private struct ActionButtonStyle: ButtonStyle {
    let tint: Color

    func makeBody(configuration: Configuration) -> some View {
        configuration.label
            .font(.headline)
            .padding(.horizontal, 18)
            .padding(.vertical, 14)
            .foregroundStyle(.white)
            .background(tint.opacity(configuration.isPressed ? 0.8 : 1.0), in: RoundedRectangle(cornerRadius: 16, style: .continuous))
            .scaleEffect(configuration.isPressed ? 0.98 : 1.0)
    }
}

private struct ModeSelector: View {
    @ObservedObject var model: AppModel

    var body: some View {
        HStack(spacing: 10) {
            modeButton(title: "Mapping", mode: "mapping")
            modeButton(title: "Localization", mode: "localization")
        }
    }

    private func modeButton(title: String, mode: String) -> some View {
        let isSelected = model.bridgeState.mode == mode

        return Button {
            guard !model.isSwitchingMode else { return }
            Task { await model.setMode(mode) }
        } label: {
            Text(title)
                .font(.headline.weight(.semibold))
                .lineLimit(1)
                .minimumScaleFactor(0.85)
                .frame(maxWidth: .infinity, minHeight: 52)
                .padding(.horizontal, 14)
        }
        .foregroundStyle(.white)
        .background(
            RoundedRectangle(cornerRadius: 18, style: .continuous)
                .fill(isSelected ? Color(red: 0.81, green: 0.63, blue: 0.27) : Color.white.opacity(0.1))
        )
        .overlay(
            RoundedRectangle(cornerRadius: 18, style: .continuous)
                .stroke(Color.white.opacity(isSelected ? 0.0 : 0.14), lineWidth: 1)
        )
        .contentShape(Rectangle())
        .disabled(model.isSwitchingMode)
    }
}

private func dashboardHeader(title: String, subtitle: String, model: AppModel) -> some View {
    VStack(alignment: .leading, spacing: 12) {
        Text(title)
            .font(.system(size: 34, weight: .bold, design: .rounded))
            .foregroundStyle(.white)
        Text(subtitle)
            .font(.headline)
            .foregroundStyle(Color.white.opacity(0.72))
        ModeSelector(model: model)
    }
    .frame(maxWidth: .infinity, alignment: .leading)
}
