use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use trouble_host::prelude::*;

/// Static channel for receiving data from BLE host (host writes to RX characteristic)
pub static DATA_CHANNEL_RX: Channel<CriticalSectionRawMutex, [u8; 64], 4> = Channel::new();

/// Static channel for sending data to BLE host (firmware writes, TX characteristic notifies)
pub static DATA_CHANNEL_TX: Channel<CriticalSectionRawMutex, [u8; 64], 4> = Channel::new();

// Custom 128-bit UUIDs for K9-Pad Data Channel
// Base: e9dc0000-7374-7265-616d-6b3970616400
#[gatt_service(uuid = "e9dc0001-7374-7265-616d-6b3970616400")]
pub(crate) struct DataChannelService {
    #[characteristic(uuid = "e9dc0002-7374-7265-616d-6b3970616400", write_without_response, value = [0; 64])]
    pub(crate) rx_from_host: [u8; 64],

    #[characteristic(uuid = "e9dc0003-7374-7265-616d-6b3970616400", read, notify, value = [0; 64])]
    pub(crate) tx_to_host: [u8; 64],
}

/// Wrapper around the data channel TX characteristic for sending notifications
pub(crate) struct BleDataChannelServer<'stack, 'server, 'conn, P: PacketPool> {
    pub(crate) tx_to_host: Characteristic<[u8; 64]>,
    pub(crate) conn: &'conn GattConnection<'stack, 'server, P>,
}

impl<'stack, 'server, 'conn, P: PacketPool> BleDataChannelServer<'stack, 'server, 'conn, P> {
    pub(crate) fn new(
        server: &super::ble_server::Server,
        conn: &'conn GattConnection<'stack, 'server, P>,
    ) -> Self {
        Self {
            tx_to_host: server.data_channel_service.tx_to_host,
            conn,
        }
    }

    /// Run the data channel TX task: receives from DATA_CHANNEL_TX and sends BLE notifications
    pub(crate) async fn run(&self) {
        loop {
            let data = DATA_CHANNEL_TX.receive().await;
            if let Err(e) = self.tx_to_host.notify(self.conn, &data).await {
                error!("Data channel TX notify error: {:?}", e);
            }
        }
    }
}
