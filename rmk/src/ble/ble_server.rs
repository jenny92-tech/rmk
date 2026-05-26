use trouble_host::prelude::*;
use usbd_hid::descriptor::{AsInputReport, SerializedDescriptor};

use super::battery_service::BatteryService;
use super::device_info::DeviceConfigurationService;
#[cfg(feature = "data_channel")]
use crate::hid::DataChannelReport;
#[cfg(feature = "host")]
use crate::hid::ViaReport;
use crate::hid::{CompositeReport, CompositeReportType, HidError, HidWriterTrait, KeyboardReport, Report};

// Used for saving the client attribute (CCCD) table. Tracks the trouble-host
// per-connection client-specific attribute buffer size.
pub(crate) const CCCD_TABLE_SIZE: usize = trouble_host::config::CLIENT_ATT_TABLE_SIZE;

// `gatt_server` compiles every member regardless of the surrounding `cfg` —
// gating an individual field with `#[cfg(feature = "host")]` doesn't work. So
// the whole struct is duplicated for every combination of optional services
// (currently host × data_channel).
#[cfg(all(feature = "host", feature = "data_channel"))]
#[gatt_server]
pub(crate) struct Server {
    pub(crate) battery_service: BatteryService,
    pub(crate) hid_service: HidService,
    pub(crate) host_service: VialService,
    pub(crate) composite_service: CompositeService,
    pub(crate) device_config_service: DeviceConfigurationService,
    pub(crate) data_channel_service: DataChannelService,
}

#[cfg(all(feature = "host", not(feature = "data_channel")))]
#[gatt_server]
pub(crate) struct Server {
    pub(crate) battery_service: BatteryService,
    pub(crate) hid_service: HidService,
    pub(crate) host_service: VialService,
    pub(crate) composite_service: CompositeService,
    pub(crate) device_config_service: DeviceConfigurationService,
}

/// GATT service exposing the Vial-over-HID protocol. The keyboard writes replies via
/// `input_data` notify; hosts push requests through `output_data`. `gatt_events_task`
/// forwards `output_data` writes into `HOST_REQUEST_CHANNEL`, and `host::run_ble_host`
/// drains `HOST_BLE_REPLY` to notify `input_data`.
#[cfg(feature = "host")]
#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
pub(crate) struct VialService {
    #[characteristic(uuid = "2a4a", read, value = [0x01, 0x01, 0x00, 0x03])]
    pub(crate) hid_info: [u8; 4],
    #[characteristic(uuid = "2a4b", read, value = ViaReport::desc().try_into().expect("Failed to convert ViaReport to [u8; 27]"))]
    pub(crate) report_map: [u8; 27],
    #[characteristic(uuid = "2a4c", write_without_response)]
    pub(crate) hid_control_point: u8,
    #[characteristic(uuid = "2a4e", read, write_without_response, value = 1)]
    pub(crate) protocol_mode: u8,
    #[descriptor(uuid = "2908", read, value = [0u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) input_data: [u8; 32],
    #[descriptor(uuid = "2908", read, value = [0u8, 2u8])]
    #[characteristic(uuid = "2a4d", read, write, write_without_response)]
    pub(crate) output_data: [u8; 32],
}

#[cfg(all(not(feature = "host"), feature = "data_channel"))]
#[gatt_server]
pub(crate) struct Server {
    pub(crate) battery_service: BatteryService,
    pub(crate) hid_service: HidService,
    pub(crate) composite_service: CompositeService,
    pub(crate) device_config_service: DeviceConfigurationService,
    pub(crate) data_channel_service: DataChannelService,
}

#[cfg(all(not(feature = "host"), not(feature = "data_channel")))]
#[gatt_server]
pub(crate) struct Server {
    pub(crate) battery_service: BatteryService,
    pub(crate) hid_service: HidService,
    pub(crate) composite_service: CompositeService,
    pub(crate) device_config_service: DeviceConfigurationService,
}

/// Vendor data-channel GATT service for the k9pad runtime protocol.
///
/// Base 128-bit UUID: `e9dc0000-7374-7265-616d-6b3970616400` — the trailing
/// ASCII bytes (`streamk9pad`-ish) namespace this service to the k9pad project.
///
/// Companion app writes 64-byte payloads to `rx_from_host` (write-without-
/// response → fed into `DATA_CHANNEL_RX` by the GATT events task) and
/// subscribes to `tx_to_host` notifications (driven by
/// `data_channel::ble::run_ble_data_channel` draining `DATA_CHANNEL_TX`).
#[cfg(feature = "data_channel")]
#[gatt_service(uuid = "e9dc0001-7374-7265-616d-6b3970616400")]
pub(crate) struct DataChannelService {
    #[characteristic(uuid = "e9dc0002-7374-7265-616d-6b3970616400", write_without_response, value = [0; 64])]
    pub(crate) rx_from_host: [u8; 64],

    #[characteristic(uuid = "e9dc0003-7374-7265-616d-6b3970616400", read, notify, value = [0; 64])]
    pub(crate) tx_to_host: [u8; 64],
}

#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
pub(crate) struct HidService {
    #[characteristic(uuid = "2a4a", read, value = [0x01, 0x01, 0x00, 0x03])]
    pub(crate) hid_info: [u8; 4],
    #[characteristic(uuid = "2a4b", read, value = KeyboardReport::desc().try_into().expect("Failed to convert KeyboardReport to [u8; 67]"))]
    pub(crate) report_map: [u8; 67],
    #[characteristic(uuid = "2a4c", write_without_response)]
    pub(crate) hid_control_point: u8,
    #[characteristic(uuid = "2a4e", read, write_without_response, value = 1)]
    pub(crate) protocol_mode: u8,
    #[descriptor(uuid = "2908", read, value = [0u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) input_keyboard: [u8; 8],
    #[descriptor(uuid = "2908", read, value = [0u8, 2u8])]
    #[characteristic(uuid = "2a4d", read, write, write_without_response)]
    pub(crate) output_keyboard: [u8; 1],
}

#[gatt_service(uuid = service::HUMAN_INTERFACE_DEVICE)]
pub(crate) struct CompositeService {
    #[characteristic(uuid = "2a4a", read, value = [0x01, 0x01, 0x00, 0x03])]
    pub(crate) hid_info: [u8; 4],
    #[characteristic(uuid = "2a4b", read, value = CompositeReport::desc().try_into().expect("Failed to convert CompositeReport to [u8; 111]"))]
    pub(crate) report_map: [u8; 111],
    #[characteristic(uuid = "2a4c", write_without_response)]
    pub(crate) hid_control_point: u8,
    #[characteristic(uuid = "2a4e", read, write_without_response, value = 1)]
    pub(crate) protocol_mode: u8,
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Mouse as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) mouse_report: [u8; 5],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::Media as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) media_report: [u8; 2],
    #[descriptor(uuid = "2908", read, value = [CompositeReportType::System as u8, 1u8])]
    #[characteristic(uuid = "2a4d", read, notify)]
    pub(crate) system_report: [u8; 1],
}

pub(crate) struct BleHidServer<'stack, 'server, 'conn, P: PacketPool> {
    input_keyboard: Characteristic<[u8; 8]>,
    mouse_report: Characteristic<[u8; 5]>,
    media_report: Characteristic<[u8; 2]>,
    system_report: Characteristic<[u8; 1]>,
    conn: &'conn GattConnection<'stack, 'server, P>,
}

impl<'stack, 'server, 'conn, P: PacketPool> BleHidServer<'stack, 'server, 'conn, P> {
    pub(crate) fn new(server: &Server, conn: &'conn GattConnection<'stack, 'server, P>) -> Self {
        Self {
            input_keyboard: server.hid_service.input_keyboard,
            mouse_report: server.composite_service.mouse_report,
            media_report: server.composite_service.media_report,
            system_report: server.composite_service.system_report,
            conn,
        }
    }

    async fn notify_report<R: AsInputReport, const N: usize>(
        &self,
        characteristic: Characteristic<[u8; N]>,
        report: &R,
    ) -> Result<usize, HidError> {
        let mut buf = [0u8; N];
        let n = report.serialize(&mut buf).map_err(|_| HidError::ReportSerializeError)?;
        characteristic.notify(self.conn, &buf).await.map_err(|e| {
            error!("Failed to notify HID report: {:?}", e);
            HidError::BleError
        })?;
        Ok(n)
    }
}

impl<P: PacketPool> HidWriterTrait for BleHidServer<'_, '_, '_, P> {
    type ReportType = Report;

    async fn write_report(&mut self, report: &Self::ReportType) -> Result<usize, HidError> {
        match report {
            Report::KeyboardReport(r) => self.notify_report(self.input_keyboard, r).await,
            Report::MouseReport(r) => self.notify_report(self.mouse_report, r).await,
            Report::MediaKeyboardReport(r) => self.notify_report(self.media_report, r).await,
            Report::SystemControlReport(r) => self.notify_report(self.system_report, r).await,
            // Plover HID over BLE is not supported: the stock HID-over-GATT service
            // has no stenography characteristic. Drop silently at the writer.
            #[cfg(feature = "steno")]
            Report::StenoReport(_) => {
                debug!("Steno chord dropped: Plover HID over BLE is not supported");
                Ok(0)
            }
        }
    }
}
