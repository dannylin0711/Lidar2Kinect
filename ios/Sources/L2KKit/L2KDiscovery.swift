// Finds receivers that advertise themselves with Bonjour (service type "_l2k._udp").
// A receiver does not have to advertise: its address can always be typed in.
import Foundation
import Network

public final class L2KDiscovery {
    public struct Receiver: Identifiable, Equatable {
        public let name: String
        public let endpoint: NWEndpoint
        public var id: String { name }

        public static func == (a: Receiver, b: Receiver) -> Bool { a.name == b.name }
    }

    /// Called on the main queue with the receivers currently visible.
    public var onChange: (([Receiver]) -> Void)?
    private var browser: NWBrowser?

    public init() {}

    public func start() {
        stop()
        let browser = NWBrowser(for: .bonjour(type: L2K.bonjourType, domain: nil), using: .udp)
        browser.browseResultsChangedHandler = { [weak self] results, _ in
            let receivers: [Receiver] = results.compactMap { result in
                guard case .service(let name, _, _, _) = result.endpoint else { return nil }
                return Receiver(name: name, endpoint: result.endpoint)
            }
            self?.onChange?(receivers.sorted { $0.name < $1.name })
        }
        browser.start(queue: .main)
        self.browser = browser
    }

    public func stop() {
        browser?.cancel()
        browser = nil
    }
}
