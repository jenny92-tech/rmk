#![doc = include_str!("../README.md")]
//! ## Feature flags
#![doc = document_features::document_features!()]
// Add docs.rs logo
#![doc(
    html_logo_url = "https://github.com/HaoboGu/rmk/blob/dad1f922f471127f5449262c4cb4a922e351bf43/docs/images/rmk_logo.svg?raw=true"
)]
// Make compiler and rust analyzer happy
#![allow(dead_code)]
#![allow(non_snake_case, non_upper_case_globals)]
#![allow(async_fn_in_trait)]
// Enable std for espidf and test
#![cfg_attr(not(test), no_std)]

// Re-export self as ::rmk for macro-generated code to work both inside and outside the crate
extern crate self as rmk;

// Include generated constants
include!(concat!(env!("OUT_DIR"), "/constants.rs"));

// This mod MUST go first, so that the others see its macros.
pub(crate) mod fmt;

use core::cell::RefCell;
use core::future::Future;
use core::sync::atomic::Ordering;

#[cfg(feature = "_ble")]
use bt_hci::{
    cmd::le::{LeReadLocalSupportedFeatures, LeSetPhy},
    controller::{ControllerCmdAsync, ControllerCmdSync},
};
use builtin_processor::wpm::WpmProcessor;
use config::RmkConfig;
#[cfg(not(feature = "_ble"))]
use descriptor::{CompositeReport, KeyboardReport};
#[cfg(not(any(cortex_m)))]
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex as RawMutex;
#[cfg(cortex_m)]
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex as RawMutex;
#[cfg(not(feature = "_no_usb"))]
use embassy_usb::driver::Driver;
use futures::FutureExt;
use hid::{HidReaderTrait, RunnableHidWriter};
use keymap::KeyMap;
use matrix::MatrixTrait;
use processor::PollingProcessor;
use rmk_types::action::{EncoderAction, KeyAction};
use rmk_types::led_indicator::LedIndicator;
use state::CONNECTION_STATE;
#[cfg(feature = "_ble")]
pub use trouble_host::prelude::*;
#[cfg(feature = "host")]
use {crate::descriptor::ViaReport, crate::hid::HidWriterTrait, crate::host::run_host_communicate_task};
#[cfg(all(not(feature = "_no_usb"), not(feature = "_ble")))]
use {
    crate::light::UsbLedReader,
    crate::usb::{UsbKeyboardWriter, add_usb_reader_writer, add_usb_writer, new_usb_builder},
};
pub use {embassy_futures, futures, heapless, rmk_macro as macros, rmk_types as types};
#[cfg(feature = "storage")]
use {embedded_storage_async::nor_flash::NorFlash as AsyncNorFlash, storage::Storage};

use crate::config::PositionalConfig;
#[cfg(feature = "vial")]
use crate::config::VialConfig;
use crate::event::{LedIndicatorEvent, publish_event};
use crate::keyboard::LOCK_LED_STATES;
use crate::state::ConnectionState;

#[cfg(feature = "_ble")]
pub mod ble;
mod boot;
pub mod builtin_processor;
pub mod channel;
pub mod combo;
pub mod config;
pub mod debounce;
pub mod descriptor;
pub mod direct_pin;
pub mod driver;
pub mod event;
pub mod fork;
pub mod helper_macro;
pub mod hid;
#[cfg(feature = "host")]
pub mod host;
pub mod input_device;
pub mod keyboard;
pub mod keyboard_macros;
pub mod keymap;
pub mod layout_macro;
pub mod light;
pub mod matrix;
pub mod morse;
pub mod processor;
#[cfg(feature = "split")]
pub mod split;
pub mod state;
#[cfg(feature = "storage")]
pub mod storage;
#[cfg(not(feature = "_no_usb"))]
pub mod usb;

// ============================================================================
// BLE Data Channel API
// ============================================================================

#[cfg(all(feature = "_ble", feature = "data_channel"))]
pub mod data_channel {
    pub use crate::ble::data_channel_service::{DATA_CHANNEL_RX, DATA_CHANNEL_TX};
}

// ============================================================================
// K9-Pad Runtime API
// ============================================================================

use core::sync::atomic::{AtomicBool, AtomicU8};
use crate::event::KeyboardEvent;

// --- KeyEvent: wraps KeyboardEvent + resolved KeyAction ---

/// Key event with resolved action, published for every key press/release
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KeyEvent {
    pub keyboard_event: KeyboardEvent,
    pub key_action: KeyAction,
}

static KEY_EVENT_CHANNEL: embassy_sync::pubsub::PubSubChannel<RawMutex, KeyEvent, 4, 2, 1> =
    embassy_sync::pubsub::PubSubChannel::new();

pub(crate) fn publish_key_event(event: KeyEvent) {
    KEY_EVENT_CHANNEL.immediate_publisher().publish_immediate(event);
}

/// Subscribe to key events (keyboard event + resolved action)
pub fn key_event_subscriber() -> embassy_sync::pubsub::Subscriber<'static, RawMutex, KeyEvent, 4, 2, 1> {
    KEY_EVENT_CHANNEL.subscriber().unwrap()
}

// --- Menu interception ---

/// When true, configured keys are intercepted (not sent to host)
pub static MENU_MODE_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Whether to intercept encoder events in menu mode
pub static MENU_INTERCEPT_ENCODER: AtomicBool = AtomicBool::new(false);

/// Menu intercept key storage (max 8 keys, 2 bytes each: row, col). 0xFF = not configured.
static MENU_INTERCEPT_KEYS: [AtomicU8; 16] = {
    const INIT: AtomicU8 = AtomicU8::new(0xFF);
    [INIT; 16]
};

/// Configure a key to be intercepted in menu mode
pub fn menu_intercept_set_key(index: usize, row: u8, col: u8) {
    if index < 8 {
        MENU_INTERCEPT_KEYS[index * 2].store(row, Ordering::Relaxed);
        MENU_INTERCEPT_KEYS[index * 2 + 1].store(col, Ordering::Relaxed);
    }
}

/// Clear a menu intercept key slot
pub fn menu_intercept_clear_key(index: usize) {
    if index < 8 {
        MENU_INTERCEPT_KEYS[index * 2].store(0xFF, Ordering::Relaxed);
        MENU_INTERCEPT_KEYS[index * 2 + 1].store(0xFF, Ordering::Relaxed);
    }
}

/// Clear all menu intercept keys
pub fn menu_intercept_clear_all() {
    for i in 0..16 {
        MENU_INTERCEPT_KEYS[i].store(0xFF, Ordering::Relaxed);
    }
}

pub(crate) fn should_intercept_key(row: u8, col: u8) -> bool {
    if !MENU_MODE_ACTIVE.load(Ordering::Relaxed) {
        return false;
    }
    for i in 0..8 {
        let r = MENU_INTERCEPT_KEYS[i * 2].load(Ordering::Relaxed);
        let c = MENU_INTERCEPT_KEYS[i * 2 + 1].load(Ordering::Relaxed);
        if r == 0xFF { continue; }
        if r == row && c == col { return true; }
    }
    false
}

pub(crate) fn should_intercept_encoder() -> bool {
    MENU_MODE_ACTIVE.load(Ordering::Relaxed) && MENU_INTERCEPT_ENCODER.load(Ordering::Relaxed)
}

// --- Deferred keys ---

/// Deferred key storage (max 8 keys). Deferred keys are intercepted but KeyEvent is still published.
static DEFERRED_KEYS: [AtomicU8; 16] = {
    const INIT: AtomicU8 = AtomicU8::new(0xFF);
    [INIT; 16]
};

pub fn deferred_key_set(index: usize, row: u8, col: u8) {
    if index < 8 {
        DEFERRED_KEYS[index * 2].store(row, Ordering::Relaxed);
        DEFERRED_KEYS[index * 2 + 1].store(col, Ordering::Relaxed);
    }
}

pub fn deferred_key_clear(index: usize) {
    if index < 8 {
        DEFERRED_KEYS[index * 2].store(0xFF, Ordering::Relaxed);
        DEFERRED_KEYS[index * 2 + 1].store(0xFF, Ordering::Relaxed);
    }
}

pub(crate) fn is_deferred_key(row: u8, col: u8) -> bool {
    for i in 0..8 {
        let r = DEFERRED_KEYS[i * 2].load(Ordering::Relaxed);
        let c = DEFERRED_KEYS[i * 2 + 1].load(Ordering::Relaxed);
        if r == 0xFF { continue; }
        if r == row && c == col { return true; }
    }
    false
}

// --- send_keycode ---

use rmk_types::keycode::KeyCode;

static PENDING_KEYCODES: embassy_sync::channel::Channel<RawMutex, (KeyCode, bool), 8> =
    embassy_sync::channel::Channel::new();

/// Manually send a keycode (press or release) from external code
pub fn send_keycode(keycode: KeyCode, pressed: bool) {
    let _ = PENDING_KEYCODES.try_send((keycode, pressed));
}

/// Process pending keycodes into HID reports (called from keyboard main loop)
pub(crate) fn process_pending_keycodes() -> bool {
    let mut processed = false;
    while let Ok((keycode, pressed)) = PENDING_KEYCODES.try_receive() {
        let mut report = crate::descriptor::KeyboardReport::default();
        match keycode {
            KeyCode::Hid(hid_code) => {
                if pressed {
                    report.keycodes[0] = hid_code as u8;
                }
            }
            _ => continue,
        }
        let _ = crate::channel::KEYBOARD_REPORT_CHANNEL.try_send(crate::hid::Report::KeyboardReport(report));
        processed = true;
    }
    processed
}

// --- set_default_layer ---

static PENDING_DEFAULT_LAYER: embassy_sync::channel::Channel<RawMutex, u8, 4> =
    embassy_sync::channel::Channel::new();

/// Switch the default keymap layer at runtime
pub fn set_default_layer(layer: u8) {
    let _ = PENDING_DEFAULT_LAYER.try_send(layer);
}

// --- BLE profile API ---

#[cfg(feature = "_ble")]
use crate::channel::BLE_PROFILE_CHANNEL;
#[cfg(feature = "_ble")]
use crate::ble::profile::BleProfileAction;

/// Switch to a specific BLE profile (0-7)
#[cfg(feature = "_ble")]
pub fn switch_ble_profile(profile: u8) {
    let _ = BLE_PROFILE_CHANNEL.try_send(BleProfileAction::SwitchProfile(profile));
}

/// Clear bonding info for the current BLE profile
#[cfg(feature = "_ble")]
pub fn clear_ble_bond() {
    let _ = BLE_PROFILE_CHANNEL.try_send(BleProfileAction::ClearProfile);
}

// ============================================================================

pub async fn initialize_keymap<'a, const ROW: usize, const COL: usize, const NUM_LAYER: usize>(
    default_keymap: &'a mut [[[KeyAction; COL]; ROW]; NUM_LAYER],
    behavior_config: &'a mut config::BehaviorConfig,
    positional_config: &'a mut PositionalConfig<ROW, COL>,
) -> RefCell<KeyMap<'a, ROW, COL, NUM_LAYER>> {
    RefCell::new(KeyMap::new(default_keymap, None, behavior_config, positional_config).await)
}

pub async fn initialize_encoder_keymap<
    'a,
    const ROW: usize,
    const COL: usize,
    const NUM_LAYER: usize,
    const NUM_ENCODER: usize,
>(
    default_keymap: &'a mut [[[KeyAction; COL]; ROW]; NUM_LAYER],
    default_encoder_map: &'a mut [[EncoderAction; NUM_ENCODER]; NUM_LAYER],
    behavior_config: &'a mut config::BehaviorConfig,
    positional_config: &'a mut PositionalConfig<ROW, COL>,
) -> RefCell<KeyMap<'a, ROW, COL, NUM_LAYER, NUM_ENCODER>> {
    RefCell::new(
        KeyMap::new(
            default_keymap,
            Some(default_encoder_map),
            behavior_config,
            positional_config,
        )
        .await,
    )
}

#[cfg(feature = "storage")]
pub async fn initialize_encoder_keymap_and_storage<
    'a,
    F: AsyncNorFlash,
    const ROW: usize,
    const COL: usize,
    const NUM_LAYER: usize,
    const NUM_ENCODER: usize,
>(
    default_keymap: &'a mut [[[KeyAction; COL]; ROW]; NUM_LAYER],
    default_encoder_map: &'a mut [[EncoderAction; NUM_ENCODER]; NUM_LAYER],
    flash: F,
    storage_config: &config::StorageConfig,
    behavior_config: &'a mut config::BehaviorConfig,
    positional_config: &'a mut PositionalConfig<ROW, COL>,
) -> (
    RefCell<KeyMap<'a, ROW, COL, NUM_LAYER, NUM_ENCODER>>,
    Storage<F, ROW, COL, NUM_LAYER, NUM_ENCODER>,
) {
    #[cfg(feature = "host")]
    {
        let mut storage = Storage::new(
            flash,
            default_keymap,
            &Some(default_encoder_map),
            storage_config,
            behavior_config,
        )
        .await;

        let keymap = RefCell::new(
            KeyMap::new_from_storage(
                default_keymap,
                Some(default_encoder_map),
                Some(&mut storage),
                behavior_config,
                positional_config,
            )
            .await,
        );
        (keymap, storage)
    }

    #[cfg(not(feature = "host"))]
    {
        let storage = Storage::new(flash, storage_config, &behavior_config).await;
        let keymap = RefCell::new(
            KeyMap::new(
                default_keymap,
                Some(default_encoder_map),
                behavior_config,
                positional_config,
            )
            .await,
        );
        (keymap, storage)
    }
}

#[cfg(feature = "storage")]
pub async fn initialize_keymap_and_storage<
    'a,
    F: AsyncNorFlash,
    const ROW: usize,
    const COL: usize,
    const NUM_LAYER: usize,
>(
    default_keymap: &'a mut [[[KeyAction; COL]; ROW]; NUM_LAYER],
    flash: F,
    storage_config: &config::StorageConfig,
    behavior_config: &'a mut config::BehaviorConfig,
    positional_config: &'a mut PositionalConfig<ROW, COL>,
) -> (
    RefCell<KeyMap<'a, ROW, COL, NUM_LAYER, 0>>,
    Storage<F, ROW, COL, NUM_LAYER, 0>,
) {
    #[cfg(feature = "host")]
    {
        let mut storage = Storage::new(flash, default_keymap, &None, storage_config, behavior_config).await;
        let keymap = RefCell::new(
            KeyMap::new_from_storage(
                default_keymap,
                None,
                Some(&mut storage),
                behavior_config,
                positional_config,
            )
            .await,
        );
        (keymap, storage)
    }

    #[cfg(not(feature = "host"))]
    {
        let storage = Storage::new(flash, storage_config, &behavior_config).await;
        let keymap = RefCell::new(KeyMap::new(default_keymap, None, behavior_config, positional_config).await);
        (keymap, storage)
    }
}

#[allow(unreachable_code)]
pub async fn run_rmk<
    #[cfg(feature = "host")] 'a,
    #[cfg(feature = "_ble")] 'b,
    #[cfg(feature = "_ble")] C: Controller + ControllerCmdAsync<LeSetPhy> + ControllerCmdSync<LeReadLocalSupportedFeatures>,
    #[cfg(feature = "storage")] F: AsyncNorFlash,
    #[cfg(not(feature = "_no_usb"))] D: Driver<'static>,
    #[cfg(any(feature = "storage", feature = "host"))] const ROW: usize,
    #[cfg(any(feature = "storage", feature = "host"))] const COL: usize,
    #[cfg(any(feature = "storage", feature = "host"))] const NUM_LAYER: usize,
    #[cfg(any(feature = "storage", feature = "host"))] const NUM_ENCODER: usize,
>(
    #[cfg(feature = "host")] keymap: &'a RefCell<KeyMap<'a, ROW, COL, NUM_LAYER, NUM_ENCODER>>,
    #[cfg(not(feature = "_no_usb"))] usb_driver: D,
    #[cfg(feature = "_ble")] stack: &'b Stack<'b, C, DefaultPacketPool>,
    #[cfg(feature = "storage")] storage: &mut Storage<F, ROW, COL, NUM_LAYER, NUM_ENCODER>,
    rmk_config: RmkConfig<'static>,
) -> ! {
    // Dispatch the keyboard runner
    #[cfg(feature = "_ble")]
    crate::ble::run_ble(
        #[cfg(feature = "host")]
        keymap,
        #[cfg(not(feature = "_no_usb"))]
        usb_driver,
        #[cfg(feature = "_ble")]
        stack,
        #[cfg(feature = "storage")]
        storage,
        rmk_config,
    )
    .await;

    // USB keyboard
    #[cfg(all(not(feature = "_no_usb"), not(feature = "_ble")))]
    {
        let mut usb_builder: embassy_usb::Builder<'_, D> = new_usb_builder(usb_driver, rmk_config.device_config);
        let keyboard_reader_writer = add_usb_reader_writer!(&mut usb_builder, KeyboardReport, 1, 8);
        let mut other_writer = add_usb_writer!(&mut usb_builder, CompositeReport, 9);
        #[cfg(feature = "host")]
        let mut host_reader_writer = add_usb_reader_writer!(&mut usb_builder, ViaReport, 32, 32);

        let (mut keyboard_reader, mut keyboard_writer) = keyboard_reader_writer.split();

        #[cfg(feature = "usb_log")]
        let logger_fut = {
            let usb_logger = crate::usb::add_usb_logger!(&mut usb_builder);
            embassy_usb_logger::with_class!(1024, log::LevelFilter::Debug, usb_logger)
        };

        #[cfg(not(feature = "usb_log"))]
        let logger_fut = async {};
        let mut usb_device = usb_builder.build();

        // Run all tasks, if one of them fails, wait 1 second and then restart
        embassy_futures::join::join(logger_fut, async {
            loop {
                let usb_task = async {
                    loop {
                        use embassy_futures::select::{Either, select};

                        use crate::usb::USB_REMOTE_WAKEUP;

                        // Run
                        usb_device.run_until_suspend().await;
                        // Suspended, wait resume or remote wakeup
                        match select(usb_device.wait_resume(), USB_REMOTE_WAKEUP.wait()).await {
                            Either::First(_) => continue,
                            Either::Second(_) => {
                                info!("USB wakeup remote");
                            }
                        }
                    }
                };

                run_keyboard(
                    #[cfg(feature = "storage")]
                    storage,
                    #[cfg(feature = "host")]
                    keymap,
                    #[cfg(feature = "host")]
                    crate::host::UsbHostReaderWriter::new(&mut host_reader_writer),
                    #[cfg(feature = "vial")]
                    rmk_config.vial_config,
                    usb_task,
                    UsbLedReader::new(&mut keyboard_reader),
                    UsbKeyboardWriter::new(&mut keyboard_writer, &mut other_writer),
                )
                .await;
            }
        })
        .await;
    }

    unreachable!("Should never reach here, wrong feature gate combination?");
}

// Run keyboard task for once
//
// Due to https://github.com/rust-lang/rust/issues/62958, storage/host struct is used now.
// The corresponding future(commented) will be used after the issue is fixed.
pub(crate) async fn run_keyboard<
    #[cfg(feature = "host")] 'a,
    R: HidReaderTrait<ReportType = LedIndicator>,
    W: RunnableHidWriter,
    #[cfg(feature = "storage")] F: AsyncNorFlash,
    #[cfg(feature = "host")] Rw: HidReaderTrait<ReportType = ViaReport> + HidWriterTrait<ReportType = ViaReport>,
    #[cfg(any(feature = "storage", feature = "host"))] const ROW: usize,
    #[cfg(any(feature = "storage", feature = "host"))] const COL: usize,
    #[cfg(any(feature = "storage", feature = "host"))] const NUM_LAYER: usize,
    #[cfg(any(feature = "storage", feature = "host"))] const NUM_ENCODER: usize,
>(
    // #[cfg(feature = "storage")] storage_task: impl Future<Output = ()>,
    #[cfg(feature = "storage")] storage: &mut Storage<F, ROW, COL, NUM_LAYER, NUM_ENCODER>,
    // #[cfg(feature = "host")] host_task: impl Future<Output = ()>,
    #[cfg(feature = "host")] keymap: &'a RefCell<KeyMap<'a, ROW, COL, NUM_LAYER, NUM_ENCODER>>,
    #[cfg(feature = "host")] reader_writer: Rw,
    #[cfg(feature = "vial")] vial_config: VialConfig<'static>,
    communication_fut: impl Future<Output = ()>,
    mut led_reader: R,
    mut keyboard_writer: W,
) {
    // The state will be changed to true after the keyboard starts running
    CONNECTION_STATE.store(ConnectionState::Connected.into(), Ordering::Release);
    let writer_fut = keyboard_writer.run_writer();
    let led_fut = async {
        loop {
            match led_reader.read_report().await {
                Ok(led_indicator) => {
                    info!("Got led indicator");
                    LOCK_LED_STATES.store(led_indicator.into_bits(), core::sync::atomic::Ordering::Relaxed);
                    publish_event(LedIndicatorEvent {
                        indicator: led_indicator,
                    });
                }
                Err(e) => {
                    error!("Read HID LED indicator error: {:?}", e);
                    embassy_time::Timer::after_millis(1000).await
                }
            }
        }
    };

    #[cfg(feature = "host")]
    let host_fut = run_host_communicate_task(
        keymap,
        reader_writer,
        #[cfg(feature = "vial")]
        vial_config,
    );
    #[cfg(feature = "storage")]
    let storage_fut = storage.run();

    let mut wpm_processor = WpmProcessor::new();

    #[cfg(feature = "storage")]
    let storage_task = core::pin::pin!(storage_fut.fuse());
    #[cfg(feature = "host")]
    let host_task = core::pin::pin!(host_fut.fuse());
    let mut communication_task = core::pin::pin!(communication_fut.fuse());
    let mut led_task = core::pin::pin!(led_fut.fuse());
    let mut writer_task = core::pin::pin!(writer_fut.fuse());

    futures::select_biased! {
        _ = communication_task => error!("Communication task has ended"),
        _ = with_feature!("storage", storage_task) => error!("Storage task has ended"),
        _ = wpm_processor.polling_loop().fuse() => error!("WPM Processor task ended"),
        _ = led_task => error!("Led task has ended"),
        _ = with_feature!("host", host_task) => error!("Host task ended"),
        _ = writer_task => error!("Writer task has ended"),
    };

    CONNECTION_STATE.store(ConnectionState::Disconnected.into(), Ordering::Release);
}
