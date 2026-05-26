use trouble_host::prelude::*;

use crate::channel::DATA_CHANNEL_TX;

/// Drains `DATA_CHANNEL_TX` and forwards each 64-byte payload as a GATT
/// notify on the data-channel `tx_to_host` characteristic. The companion
/// BLE write handler in `ble::mod::gatt_events_task` pushes received
/// payloads into `DATA_CHANNEL_RX`.
pub(crate) async fn run_ble_data_channel<P: PacketPool>(
    tx_to_host: Characteristic<[u8; 64]>,
    conn: &GattConnection<'_, '_, P>,
) -> ! {
    DATA_CHANNEL_TX.clear();
    loop {
        let payload = DATA_CHANNEL_TX.receive().await;
        if let Err(e) = tx_to_host.notify(conn, &payload, true).await {
            error!("Data channel notify error: {:?}", e);
        }
    }
}
