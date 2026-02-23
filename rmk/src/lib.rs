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
// Data Channel (transport-agnostic)
// ============================================================================

#[cfg(feature = "data_channel")]
pub mod data_channel;

// ============================================================================
// Controller Event API (控制器事件)
// ============================================================================
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16};

/// Whether a deferred key's hold threshold was reached
#[cfg(feature = "controller")]
#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DeferredEventState {
    /// Normal press/release event (tap decision pending or non-deferred key)
    #[default]
    Normal,
    /// Hold threshold reached — controller should handle as hold action
    HoldActivated,
}

/// Key event published to controller subscribers
#[cfg(feature = "controller")]
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KeyEvent {
    pub keyboard_event: event::KeyboardEvent,
    pub key_action: rmk_types::action::KeyAction,
    pub deferred_state: DeferredEventState,
}

#[cfg(feature = "controller")]
static KEY_EVENT_CHANNEL: embassy_sync::pubsub::PubSubChannel<RawMutex, KeyEvent, 4, 2, 1> =
    embassy_sync::pubsub::PubSubChannel::new();

#[cfg(feature = "controller")]
pub(crate) fn publish_controller_event(event: KeyEvent) {
    KEY_EVENT_CHANNEL.immediate_publisher().publish_immediate(event);
}

/// Subscribe to key events (keyboard event + resolved action)
#[cfg(feature = "controller")]
pub fn key_event_subscriber() -> embassy_sync::pubsub::Subscriber<'static, RawMutex, KeyEvent, 4, 2, 1> {
    KEY_EVENT_CHANNEL.subscriber().unwrap()
}

// ============================================================================
// Menu Intercept Feature (菜单拦截功能)
//
// 允许在菜单模式下拦截指定按键，不发送到主机
// Allows intercepting specific keys in menu mode, preventing them from being sent to host
// ============================================================================

/// 菜单模式激活标志
/// 当为 true 时，MENU_INTERCEPT_KEYS 中的按键不会发送到主机
///
/// Menu mode active flag.
/// When true, keys configured in menu_intercept_set_key() won't be sent to host.
///
/// # Example
/// ```ignore
/// // 进入菜单
/// rmk::MENU_MODE_ACTIVE.store(true, core::sync::atomic::Ordering::Relaxed);
/// // 退出菜单
/// rmk::MENU_MODE_ACTIVE.store(false, core::sync::atomic::Ordering::Relaxed);
/// ```
#[cfg(feature = "controller")]
pub static MENU_MODE_ACTIVE: AtomicBool = AtomicBool::new(false);

/// 菜单模式下是否拦截编码器事件
/// Whether to intercept encoder events in menu mode
#[cfg(feature = "controller")]
pub static MENU_INTERCEPT_ENCODER: AtomicBool = AtomicBool::new(false);

/// 菜单拦截按键存储 (最多 8 个按键，每个按键 2 字节: row, col)
/// 0xFF 表示未配置
///
/// Storage for menu intercept keys (max 8 keys, 2 bytes each: row, col)
/// 0xFF means not configured
#[cfg(feature = "controller")]
static MENU_INTERCEPT_KEYS: [AtomicU8; 16] = [
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
];

/// 配置菜单拦截按键 (运行时调用)
/// Configure a key to be intercepted in menu mode (call at runtime)
///
/// # Arguments
/// * `index` - 按键槽位 (0-7) / Key slot (0-7)
/// * `row` - 按键行号 / Key row
/// * `col` - 按键列号 / Key column
///
/// # Example
/// ```ignore
/// // 在初始化时配置 / Configure at initialization
/// rmk::menu_intercept_set_key(0, 0, 3); // SW1 at ROW0/COL3
/// rmk::menu_intercept_set_key(1, 0, 2); // Confirm key at ROW0/COL2
/// ```
#[cfg(feature = "controller")]
pub fn menu_intercept_set_key(index: usize, row: u8, col: u8) {
    if index < 8 {
        MENU_INTERCEPT_KEYS[index * 2].store(row, Ordering::Relaxed);
        MENU_INTERCEPT_KEYS[index * 2 + 1].store(col, Ordering::Relaxed);
    }
}

/// 清除指定位置的拦截配置
/// Clear intercept configuration at specified index
#[cfg(feature = "controller")]
pub fn menu_intercept_clear_key(index: usize) {
    if index < 8 {
        MENU_INTERCEPT_KEYS[index * 2].store(0xFF, Ordering::Relaxed);
        MENU_INTERCEPT_KEYS[index * 2 + 1].store(0xFF, Ordering::Relaxed);
    }
}

/// 清除所有拦截配置
/// Clear all intercept configurations
#[cfg(feature = "controller")]
pub fn menu_intercept_clear_all() {
    for i in 0..16 {
        MENU_INTERCEPT_KEYS[i].store(0xFF, Ordering::Relaxed);
    }
}

/// 检查按键是否应该被菜单拦截
/// Check if a key should be intercepted by menu
#[cfg(feature = "controller")]
#[inline]
pub fn should_intercept_key(row: u8, col: u8) -> bool {
    if !MENU_MODE_ACTIVE.load(Ordering::Relaxed) {
        return false;
    }
    for i in 0..8 {
        let r = MENU_INTERCEPT_KEYS[i * 2].load(Ordering::Relaxed);
        let c = MENU_INTERCEPT_KEYS[i * 2 + 1].load(Ordering::Relaxed);
        if r == row && c == col {
            return true;
        }
    }
    false
}

/// 检查编码器是否应该被菜单拦截
/// Check if encoder should be intercepted by menu
#[cfg(feature = "controller")]
#[inline]
pub fn should_intercept_encoder() -> bool {
    MENU_MODE_ACTIVE.load(Ordering::Relaxed)
        && MENU_INTERCEPT_ENCODER.load(Ordering::Relaxed)
}

// ============================================================================
// 延迟按键功能 (Deferred Key Feature)
//
// 延迟按键会被拦截（不发送到主机），但 KeyEvent 正常发布给控制器
// 控制器判断后可调用 send_keycode() 手动发送
// ============================================================================

/// 延迟按键存储（最多 8 个，每个 2 字节: row, col）
///
/// Storage for deferred keys (max 8 keys, 2 bytes each: row, col)
/// 0xFF means not configured
#[cfg(feature = "controller")]
static DEFERRED_KEYS: [AtomicU8; 16] = [
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
    AtomicU8::new(0xFF), AtomicU8::new(0xFF),
];

/// 配置延迟按键
///
/// 延迟按键会被拦截但 KeyEvent 仍发布给控制器，
/// 控制器可调用 send_keycode() 决定是否发送
///
/// # Example
/// ```ignore
/// // SW1 设为延迟按键，用于长按/短按检测
/// rmk::deferred_key_set(0, 0, 3);
/// ```
#[cfg(feature = "controller")]
pub fn deferred_key_set(index: usize, row: u8, col: u8) {
    if index < 8 {
        DEFERRED_KEYS[index * 2].store(row, Ordering::Relaxed);
        DEFERRED_KEYS[index * 2 + 1].store(col, Ordering::Relaxed);
    }
}

/// 清除延迟按键配置
/// Clear deferred key configuration
#[cfg(feature = "controller")]
pub fn deferred_key_clear(index: usize) {
    if index < 8 {
        DEFERRED_KEYS[index * 2].store(0xFF, Ordering::Relaxed);
        DEFERRED_KEYS[index * 2 + 1].store(0xFF, Ordering::Relaxed);
        DEFERRED_HOLD_THRESHOLDS[index].store(0, Ordering::Relaxed);
    }
}

/// Hold 阈值存储（每个 slot 一个 u16，单位 ms，0 = 旧行为/纯延迟）
///
/// Per-slot hold threshold in ms. 0 means legacy behavior (fully deferred).
#[cfg(feature = "controller")]
static DEFERRED_HOLD_THRESHOLDS: [AtomicU16; 8] = [
    AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0),
    AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0), AtomicU16::new(0),
];

/// 配置延迟按键（带 tap 支持）
///
/// 与 `deferred_key_set` 相比，额外指定 hold 阈值：
/// - 按住超过 hold_threshold_ms → RMK 通知 controller `HoldActivated`
/// - 在阈值内释放 → RMK 自动发送 tap（走正常 HID 管线）
///
/// # Example
/// ```ignore
/// rmk::deferred_key_set_with_tap(0, 0, 3, 500); // SW1: 500ms hold threshold
/// ```
#[cfg(feature = "controller")]
pub fn deferred_key_set_with_tap(index: usize, row: u8, col: u8, hold_threshold_ms: u16) {
    if index < 8 {
        DEFERRED_KEYS[index * 2].store(row, Ordering::Relaxed);
        DEFERRED_KEYS[index * 2 + 1].store(col, Ordering::Relaxed);
        DEFERRED_HOLD_THRESHOLDS[index].store(hold_threshold_ms, Ordering::Relaxed);
    }
}

/// 查询按键的 hold 阈值（0 = 旧行为）
/// Query hold threshold for a key position (0 = legacy behavior)
#[cfg(feature = "controller")]
#[inline]
pub(crate) fn deferred_key_hold_threshold(row: u8, col: u8) -> u16 {
    for i in 0..8 {
        let r = DEFERRED_KEYS[i * 2].load(Ordering::Relaxed);
        let c = DEFERRED_KEYS[i * 2 + 1].load(Ordering::Relaxed);
        if r == row && c == col {
            return DEFERRED_HOLD_THRESHOLDS[i].load(Ordering::Relaxed);
        }
    }
    0
}

/// 检查是否是延迟按键
/// Check if a key is a deferred key
#[cfg(feature = "controller")]
#[inline]
pub fn is_deferred_key(row: u8, col: u8) -> bool {
    for i in 0..8 {
        let r = DEFERRED_KEYS[i * 2].load(Ordering::Relaxed);
        let c = DEFERRED_KEYS[i * 2 + 1].load(Ordering::Relaxed);
        if r == row && c == col {
            return true;
        }
    }
    false
}

// ============================================================================
// 手动发送按键功能 (Manual Key Send)
// ============================================================================

use channel::KEYBOARD_REPORT_CHANNEL;
use hid::Report;
use descriptor::KeyboardReport;
use rmk_types::keycode::KeyCode;

/// 待发送的按键队列
/// Channel for pending keycodes to be sent
#[cfg(feature = "controller")]
static PENDING_KEYCODES: embassy_sync::channel::Channel<RawMutex, (KeyCode, bool), 8> = embassy_sync::channel::Channel::new();

/// 手动发送按键
///
/// 控制器调用此函数发送按键到主机
///
/// # Arguments
/// * `keycode` - 要发送的键码 / Keycode to send
/// * `pressed` - true=按下, false=释放 / true=press, false=release
///
/// # Example
/// ```ignore
/// // 发送 ESC 按下和释放
/// rmk::send_keycode(KeyCode::Escape, true);
/// rmk::send_keycode(KeyCode::Escape, false);
/// ```
#[cfg(feature = "controller")]
pub fn send_keycode(keycode: KeyCode, pressed: bool) {
    let _ = PENDING_KEYCODES.try_send((keycode, pressed));
}

/// 处理待发送的按键（在键盘主循环中调用）
/// Process pending keycodes (called in keyboard main loop)
///
/// 返回是否有按键被处理 / Returns whether any key was processed
#[cfg(feature = "controller")]
pub(crate) fn process_pending_keycodes() -> bool {
    use crate::hid::HidWriterTrait;
    let mut processed = false;
    while let Ok((keycode, pressed)) = PENDING_KEYCODES.try_receive() {
        // 构建并发送 HID 报告
        let mut report = KeyboardReport::default();
        if pressed {
            // 按下：添加键码
            match keycode {
                KeyCode::Hid(hid_code) => {
                    report.keycodes[0] = hid_code as u8;
                }
                KeyCode::Consumer(consumer_key) => {
                    if let Some(hid_code) = consumer_key.to_hid_keycode() {
                        report.keycodes[0] = hid_code as u8;
                    }
                }
                KeyCode::SystemControl(system_key) => {
                    if let Some(hid_code) = system_key.to_hid_keycode() {
                        report.keycodes[0] = hid_code as u8;
                    }
                }
            }
        }
        // 发送到报告通道
        let _ = KEYBOARD_REPORT_CHANNEL.try_send(Report::KeyboardReport(report));
        processed = true;
    }
    processed
}

// ============================================================================
// BLE Profile 公共 API (BLE Profile Public API)
// ============================================================================

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

/// 切换到指定 BLE Profile
///
/// # Arguments
/// * `profile` - Profile 编号 (0-7)
///
/// # Example
/// ```ignore
/// rmk::switch_ble_profile(1); // 切换到 Profile 1
/// ```
#[cfg(feature = "_ble")]
pub fn switch_ble_profile(profile: u8) {
    let _ = BLE_PROFILE_CHANNEL.try_send(BleProfileAction::SwitchProfile(profile));
}

/// 清除当前 BLE Profile 的配对信息
///
/// # Example
/// ```ignore
/// rmk::clear_ble_bond();
/// ```
#[cfg(feature = "_ble")]
pub fn clear_ble_bond() {
    let _ = BLE_PROFILE_CHANNEL.try_send(BleProfileAction::ClearProfile);
}

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
    #[cfg(feature = "_custom_tasks")] extra_task: impl Future<Output = ()>,
) -> ! {
    // Dispatch the keyboard runner
    #[cfg(feature = "_ble")]
    {
        let extra_task_fut = async {
            #[cfg(feature = "_custom_tasks")]
            extra_task.await;
            #[cfg(not(feature = "_custom_tasks"))]
            core::future::pending::<()>().await;
        };
        
        embassy_futures::join::join(
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
            ),
            extra_task_fut,
        ).await;
    }

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
        let extra_task_fut = async {
            #[cfg(feature = "_custom_tasks")]
            extra_task.await;
            #[cfg(not(feature = "_custom_tasks"))]
            core::future::pending::<()>().await;
        };
        
        embassy_futures::join::join3(logger_fut, async {
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
        }, extra_task_fut)
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
