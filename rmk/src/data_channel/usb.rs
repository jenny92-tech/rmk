use embassy_futures::select::{Either, select};
use embassy_usb::class::hid::{HidReader, HidWriter, ReadError};
use embassy_usb::driver::{Driver, EndpointError};

use crate::channel::{DATA_CHANNEL_RX, DATA_CHANNEL_TX};

/// Drives the USB HID vendor endpoint for the data channel.
///
/// Two concurrent halves multiplexed with `select`:
/// - `reader.read` → push the 64-byte OUT report into `DATA_CHANNEL_RX`
/// - `DATA_CHANNEL_TX.receive` → write the 64-byte payload as an IN report
///
/// Mirrors the structure of `host::usb::run_usb_host` but with a separate
/// 64-byte interface and the simpler untagged channels.
pub(crate) async fn run_usb_data_channel<'d, D: Driver<'d>>(
    reader: &mut HidReader<'d, D, 64>,
    writer: &mut HidWriter<'d, D, 64>,
) {
    loop {
        let mut buf = [0u8; 64];
        match select(reader.read(&mut buf), DATA_CHANNEL_TX.receive()).await {
            Either::First(Ok(_)) => {
                // Drop on full — bursty hosts shouldn't back-pressure the
                // USB read task. App is expected to keep up.
                let _ = DATA_CHANNEL_RX.try_send(buf);
            }
            Either::First(Err(ReadError::Disabled)) => {
                // Endpoint disabled — wait for re-enable on the next iteration.
                embassy_time::Timer::after_millis(50).await;
            }
            Either::First(Err(e)) => error!("USB data channel read error: {:?}", e),
            Either::Second(payload) => match writer.write(&payload).await {
                Ok(()) => {}
                Err(EndpointError::Disabled) => {
                    embassy_time::Timer::after_millis(50).await;
                }
                Err(e) => error!("USB data channel write error: {:?}", e),
            },
        }
    }
}
