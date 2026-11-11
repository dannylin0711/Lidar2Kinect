import L2KKit
import L2KShared
import SwiftUI

struct ContentView: View {
    @EnvironmentObject private var model: SensorModel

    var body: some View {
        GeometryReader { geometry in
            if geometry.size.width > geometry.size.height {
                HStack(alignment: .top, spacing: 16) {
                    previewColumn
                    ScrollView { controls }.frame(width: min(380, geometry.size.width * 0.4))
                }
                .padding(16)
            } else {
                ScrollView {
                    VStack(spacing: 16) {
                        previewColumn
                        controls
                    }
                    .padding(16)
                }
            }
        }
        .background(Color.black.ignoresSafeArea())
    }

    private var previewColumn: some View {
        VStack(spacing: 8) {
            LivePreview(live: model.live, depth: model.previewMode == .depth)
            Picker("Preview", selection: $model.previewMode) {
                ForEach(SensorModel.PreviewMode.allCases) { Text($0.title).tag($0) }
            }
            .pickerStyle(.segmented)
        }
    }

    private var controls: some View {
        VStack(alignment: .leading, spacing: 16) {
            sensorCard
            receiverCard
            LiveDiagnostics(live: model.live, model: model)
            if !model.deviceSupport.isEmpty { supportCard }
            Text("Lidar2Kinect \(Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "")")
                .font(.caption2).foregroundStyle(.tertiary)
        }
    }

    // MARK: - device support

    /// What ARKit reports it can do on this device: explains which sources and streams are available.
    private var supportCard: some View {
        Card(title: "This device") {
            ForEach(model.deviceSupport, id: \.label) { item in
                HStack(alignment: .firstTextBaseline) {
                    Image(systemName: item.supported ? "checkmark.circle.fill" : "xmark.circle")
                        .foregroundStyle(item.supported ? .green : .secondary)
                    Text(item.label).font(.footnote)
                    Spacer()
                }
            }
        }
    }

    // MARK: - sensor

    private var sensorCard: some View {
        Card(title: "Sensor") {
            if model.cameraAvailable {
                CameraMenu(sources: model.availableSources, choice: model.sourceChoice) { model.selectSource($0) }
                    .equatable()
                if model.sourceChoice == .front {
                    Text("The camera on the screen's side, for playing in front of this device. No depth sensor: distances are estimated from the picture.")
                        .font(.footnote).foregroundStyle(.secondary)
                } else if model.sourceChoice == .ultrawide {
                    Text("The rear ultra-wide camera, for standing close. No depth sensor: distances are estimated from the picture.")
                        .font(.footnote).foregroundStyle(.secondary)
                }
            } else {
                Text("This device has no body tracking camera, so the test figure is used.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
            Toggle("Upper body only", isOn: $model.upperBodyOnly)
            if model.upperBodyOnly {
                Text("For a camera that sees you from the waist up. Your arms and head are tracked; the legs are filled in standing still, so foot and step notes will miss.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
            Toggle("Best position for Dance Evolution", isOn: $model.normalize)
            if model.normalize {
                Picker("Players in the game", selection: $model.normalizePlayers) {
                    Text("1 player").tag(1)
                    Text("2 players").tag(2)
                }
                .pickerStyle(.segmented)
            }
            Text(model.normalize
                 ? (model.normalizePlayers == 2
                    ? "The game is told two players stand 0.61 m either side of the middle of its play area, 2.5 m from a level sensor, each on the side they really stand on. Steps and leans are sent as they are; where you stand in the room is not."
                    : "The game is told you stand in the middle of its play area, 2.5 m from a level sensor, wherever you really are and however the device is tilted. Steps and leans are sent as they are; where you stand in the room is not. Other people in view are not sent.")
                 : "Off: you are reported where you really stand, as seen from this device.")
                .font(.footnote).foregroundStyle(.secondary)
            Toggle("Fit me into the game's play area", isOn: $model.fitPlayArea).disabled(model.normalize)
            Text(model.fitPlayArea
                 ? "If you stand nearer than 1.8 m, farther than 3.2 m or more than 1.25 m to one side, the game is told you are just inside those limits. Your pose is not changed."
                 : "You are reported where you really stand. The game ignores a player outside 1.8–3.2 m from the sensor or more than 1.25 m to one side.")
                .font(.footnote).foregroundStyle(.secondary)
            Toggle("Keep the same player", isOn: $model.keepPlayer)
            Text(model.keepPlayer
                 ? "A player who is lost and comes back is the same player to the game. A gap of up to half a second is not shown to the game at all."
                 : "A player lost for more than half a second comes back as a new player, as with a real Kinect.")
                .font(.footnote).foregroundStyle(.secondary)
            if let error = model.sourceError {
                Text(error).font(.footnote).foregroundStyle(.red)
            }
            LiveReadings(live: model.live, normalize: model.normalize, fitPlayArea: model.fitPlayArea)
        }
    }

    // MARK: - receiver

    private var receiverCard: some View {
        Card(title: "Receiver") {
            HStack {
                TextField("PC address, e.g. 192.168.1.20", text: $model.host)
                    .textFieldStyle(.roundedBorder)
                    .keyboardType(.numbersAndPunctuation)
                    .textInputAutocapitalization(.never)
                    .autocorrectionDisabled()
                TextField("Port", text: $model.port)
                    .textFieldStyle(.roundedBorder)
                    .keyboardType(.numberPad)
                    .frame(width: 76)
            }
            HStack {
                if model.senderStatus.state == .idle {
                    Button("Connect") { model.connect() }
                        .buttonStyle(.borderedProminent)
                        .disabled(model.host.trimmingCharacters(in: .whitespaces).isEmpty)
                } else {
                    Button("Disconnect", role: .destructive) { model.disconnect() }
                        .buttonStyle(.bordered)
                }
                Spacer()
                stateLabel
            }
            Toggle("Send the camera picture", isOn: $model.sendColor)
            if !model.sendColor {
                Text("The camera picture stays on this device. The game still gets the skeleton and the depth image; where it shows the player's camera image it has nothing to show.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
            if model.senderStatus.state == .streaming {
                Row("Receiver", model.senderStatus.receiverName)
                Row("Sending", streamNames(model.senderStatus.streams))
                if let rtt = model.senderStatus.roundTripMs {
                    Row("Round trip", String(format: "%.1f ms", rtt))
                }
            }
            if !model.receivers.isEmpty {
                Divider()
                Text("Found on this network").font(.footnote).foregroundStyle(.secondary)
                ForEach(model.receivers) { receiver in
                    Button {
                        model.connect(to: receiver)
                    } label: {
                        Label(receiver.name, systemImage: "desktopcomputer")
                    }
                }
            }
        }
    }

    private var stateLabel: some View {
        let (text, color): (String, Color) = {
            switch model.senderStatus.state {
            case .idle: return ("Not connected", .secondary)
            case .connecting: return ("Connecting", .yellow)
            case .waitingForReceiver: return ("Waiting for the receiver", .yellow)
            case .streaming: return ("Streaming", .green)
            case .failed(let reason): return (reason, .red)
            }
        }()
        return Text(text).font(.footnote).foregroundStyle(color).lineLimit(2)
    }

    private func streamNames(_ streams: UInt32) -> String {
        var names: [String] = []
        if streams & L2K.streamSkeleton != 0 { names.append("skeleton") }
        if streams & L2K.streamDepth != 0 { names.append("depth") }
        if streams & L2K.streamColor != 0 { names.append("colour") }
        return names.isEmpty ? "nothing" : names.joined(separator: ", ")
    }
}

/// The camera choice. Equatable, and used with `.equatable()`: it is only rebuilt when the choice or the list changes,
/// so its menu stays as it is while the rest of the page is redrawn.
private struct CameraMenu: View, Equatable {
    let sources: [SensorModel.SourceChoice]
    let choice: SensorModel.SourceChoice
    let select: (SensorModel.SourceChoice) -> Void

    nonisolated static func == (a: CameraMenu, b: CameraMenu) -> Bool { a.sources == b.sources && a.choice == b.choice }

    var body: some View {
        HStack {
            Text("Camera")
            Spacer()
            Picker("Camera", selection: Binding(get: { choice }, set: { select($0) })) {
                ForEach(sources) { Text($0.title).tag($0) }
            }
            .pickerStyle(.menu)
        }
    }
}

/// The preview picture with its caption: redrawn with every picture.
private struct LivePreview: View {
    @ObservedObject var live: LiveState
    let depth: Bool

    var body: some View {
        SensorPreview(image: live.preview, skeleton: live.skeleton, zoom: live.shownZoom, pictureSkeleton: live.pictureSkeleton,
                      gameView: live.gameView)
        Text(depth
             ? "Depth: bright = near, blue = outside the Kinect's 0.8–4 m, green = person, black = no reading. Skeleton: orange = your left, blue = your right."
             : "Skeleton: orange = your left side, blue = your right side."
                + (live.shownZoom > 1 ? " The picture is everything the camera sees; the dashed frame is what the game's Kinect image shows." : ""))
            .font(.caption2).foregroundStyle(.secondary).frame(maxWidth: .infinity, alignment: .leading)
    }
}

/// The source's status and what it measures right now: redrawn with every picture.
private struct LiveReadings: View {
    @ObservedObject var live: LiveState
    let normalize: Bool
    let fitPlayArea: Bool

    var body: some View {
        Text(live.sourceStatus).font(.footnote).foregroundStyle(.secondary)
        if let skeleton = live.skeleton {
            let tracked = skeleton.trackedSkeletons.count
            Row("Output", String(format: "%.0f fps", live.frameRate))
            Row("Bodies tracked", "\(tracked)")
            Row("Tilt", String(format: "%+.0f°", skeleton.elevationDegrees.rounded() + 0)) // + 0 turns -0 into 0
            Row("Height above floor", skeleton.flags & UInt32(L2K_FRAME_FLOOR_VALID) != 0
                ? String(format: "%.2f m", skeleton.floor_plane.w) : "not known yet")
            if let body = skeleton.trackedSkeletons.first?.skeleton {
                let real = live.realDistance ?? body.position.z
                Row("Distance", String(format: "%.2f m", real))
                if normalize {
                    Text(String(format: "Reported to the game at %.2f m.", body.position.z))
                        .font(.footnote).foregroundStyle(.secondary)
                } else if real < L2KPipeline.playAreaNear || real > L2KPipeline.playAreaFar {
                    Text(fitPlayArea
                         ? String(format: "The game only takes a player 1.8–3.2 m away. You are reported to it at %.2f m.", body.position.z)
                         : "The game only takes a player 1.8–3.2 m away: it will not pick you up here.")
                        .font(.footnote).foregroundStyle(fitPlayArea ? Color.secondary : Color.orange)
                }
            }
        }
    }
}

/// What the sensors deliver right now. "Copy" puts all of it, with the device support list, on the clipboard.
private struct LiveDiagnostics: View {
    @ObservedObject var live: LiveState
    @ObservedObject var model: SensorModel

    var body: some View {
        if !live.diagnostics.isEmpty {
            Card(title: "Live diagnostics") {
                ForEach(live.diagnostics, id: \.self) { line in
                    Text(line).font(.caption).monospacedDigit().foregroundStyle(.secondary)
                }
                Button("Copy diagnostics") {
                    var lines = ["Lidar2Kinect \(Bundle.main.infoDictionary?["CFBundleShortVersionString"] as? String ?? "") on \(UIDevice.current.name)",
                                 "Source: \(model.sourceChoice.title)\(model.upperBodyOnly ? ", upper body only" : "")",
                                 "Status: \(live.sourceStatus)"]
                    lines += live.diagnostics
                    lines += model.deviceSupport.map { "\($0.supported ? "yes" : "no ")  \($0.label)" }
                    UIPasteboard.general.string = lines.joined(separator: "\n")
                }
                .font(.footnote)
                Divider()
                HStack {
                    Button("Record 10 s") { model.record() }
                        .font(.footnote)
                        .disabled({ if case .recording = model.recording { return true } else { return false } }())
                    Spacer()
                    Text(recordingText).font(.footnote).foregroundStyle(.secondary)
                }
                Text("Saves ten seconds of the camera to this app's files, to test the body tracking against it on the Mac.")
                    .font(.caption2).foregroundStyle(.tertiary)
            }
        }
    }

    private var recordingText: String {
        switch model.recording {
        case .idle: return ""
        case .recording(let left): return "Recording, \(left) s left"
        case .saved(let name): return "Saved \(name)"
        case .failed(let reason): return reason
        }
    }
}

private struct Card<Content: View>: View {
    let title: String
    @ViewBuilder var content: Content

    var body: some View {
        VStack(alignment: .leading, spacing: 10) {
            Text(title).font(.headline)
            content
        }
        .padding(14)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(white: 0.12), in: RoundedRectangle(cornerRadius: 14))
    }
}

private struct Row: View {
    let label: String
    let value: String

    init(_ label: String, _ value: String) {
        self.label = label
        self.value = value
    }

    var body: some View {
        HStack {
            Text(label).foregroundStyle(.secondary)
            Spacer()
            Text(value).monospacedDigit()
        }
        .font(.subheadline)
    }
}
