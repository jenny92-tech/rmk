// INPUT:  embassy_sync, embassy_usb (optional), embassy_futures (optional)
// OUTPUT: DATA_CHANNEL_RX/TX statics, run_usb_data_channel bridge
// POS:    Public data channel infrastructure — transport-agnostic channel statics + USB HID bridge

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;

/// Static channel for receiving data from host (host → firmware).
/// Used by both BLE GATT service and USB HID bridge.
pub static DATA_CHANNEL_RX: Channel<CriticalSectionRawMutex, [u8; 64], 4> = Channel::new();

/// Static channel for sending data to host (firmware → host).
/// Used by both BLE GATT service and USB HID bridge.
pub static DATA_CHANNEL_TX: Channel<CriticalSectionRawMutex, [u8; 64], 4> = Channel::new();

/// USB HID ↔ DATA_CHANNEL bridge (mirrors BleDataChannelServer::run).
///
/// Runs two concurrent loops:
/// - reader.read → DATA_CHANNEL_RX (host → firmware)
/// - DATA_CHANNEL_TX.receive → writer.write (firmware → host)
#[cfg(not(feature = "_no_usb"))]
pub(crate) async fn run_usb_data_channel<'d, D: embassy_usb::driver::Driver<'d>>(
    reader: &mut embassy_usb::class::hid::HidReader<'d, D, 64>,
    writer: &mut embassy_usb::class::hid::HidWriter<'d, D, 64>,
) {
    embassy_futures::select::select(
        async {
            loop {
                let mut buf = [0u8; 64];
                match reader.read(&mut buf).await {
                    Ok(_n) => {
                        let _ = DATA_CHANNEL_RX.try_send(buf);
                    }
                    Err(_e) => {
                        error!("USB data channel read error: {:?}", _e);
                        embassy_time::Timer::after_millis(100).await;
                    }
                }
            }
        },
        async {
            loop {
                let data = DATA_CHANNEL_TX.receive().await;
                if let Err(_e) = writer.write(&data).await {
                    error!("USB data channel write error: {:?}", _e);
                    embassy_time::Timer::after_millis(100).await;
                }
            }
        },
    )
    .await;
}
