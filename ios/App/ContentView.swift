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
            SensorPreview(image: model.preview, skeleton: model.skeleton)
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
            if !model.deviceSupport.isEmpty { supportCard }
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
                Picker("Source", selection: Binding(get: { model.sourceChoice }, set: { model.selectSource($0) })) {
                    ForEach(model.availableSources) { Text($0.title).tag($0) }
                }
                .pickerStyle(.segmented)
            } else {
                Text("This device has no body tracking camera, so the test figure is used.")
                    .font(.footnote).foregroundStyle(.secondary)
            }
            if let error = model.sourceError {
                Text(error).font(.footnote).foregroundStyle(.red)
            }
            Text(model.sourceStatus).font(.footnote).foregroundStyle(.secondary)
            if let skeleton = model.skeleton {
                let tracked = skeleton.trackedSkeletons.count
                Row("Output", String(format: "%.0f fps", model.frameRate))
                Row("Bodies tracked", "\(tracked)")
                Row("Tilt", String(format: "%+.0f°", skeleton.elevationDegrees.rounded() + 0)) // + 0 turns -0 into 0
                Row("Height above floor", skeleton.flags & UInt32(L2K_FRAME_FLOOR_VALID) != 0
                    ? String(format: "%.2f m", skeleton.floor_plane.w) : "not known yet")
                if let body = skeleton.trackedSkeletons.first?.skeleton {
                    Row("Distance", String(format: "%.2f m", body.position.z))
                }
            }
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
