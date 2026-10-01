//! Runtime controller API for companion apps (k9pad-style).
//!
//! Lets a firmware app:
//! - configure a small set of *menu intercept* slots that swallow press/release
//!   without delivering them to the host while the app is in menu mode
//! - configure a small set of *deferred* slots: the firmware never dispatches
//!   their keymap action, the app decides what to send
//! - inject manual keystrokes onto the active transport via [`send_keycode`]
//! - switch the default layer / BLE profile at runtime ([`set_default_layer`],
//!   [`switch_ble_profile`], [`clear_ble_bond`])
//!
//! Key events themselves are observed through upstream's
//! [`crate::event::KeyboardEvent`] subscriber: the matrix publishes every event
//! before the keyboard task applies the intercept/deferred filters, so
//! swallowed keys are still visible to the app.
//!
//! Everything in this module is gated by the `controller` feature. The state
//! lives in static atomics so there is no per-instance bookkeeping — the
//! firmware app owns the API surface as plain free functions.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_sync::channel::Channel;
use rmk_types::keycode::KeyCode;

use crate::RawMutex;

// ---------------------------------------------------------------------------
// Menu intercept
// ---------------------------------------------------------------------------

/// While `true`, keys configured via [`menu_intercept_set_key`] are swallowed
/// on the host side (the firmware does NOT send their press/release).
pub static MENU_MODE_ACTIVE: AtomicBool = AtomicBool::new(false);

/// While `true` AND `MENU_MODE_ACTIVE` is also `true`, encoder events are
/// swallowed on the host side too.
pub static MENU_INTERCEPT_ENCODER: AtomicBool = AtomicBool::new(false);

const MENU_SLOTS: usize = 8;
const SLOT_EMPTY: u8 = 0xFF;

// Flat (row,col) pairs — `slot_i` lives at `[i*2]` (row) and `[i*2+1]` (col).
static MENU_INTERCEPT_KEYS: [AtomicU8; MENU_SLOTS * 2] = [const { AtomicU8::new(SLOT_EMPTY) }; MENU_SLOTS * 2];

/// Configure slot `index` (0..8) to intercept the key at `(row, col)`. Indices
/// outside the range are ignored — at most 8 distinct keys can be intercepted.
pub fn menu_intercept_set_key(index: usize, row: u8, col: u8) {
    if index < MENU_SLOTS {
        MENU_INTERCEPT_KEYS[index * 2].store(row, Ordering::Relaxed);
        MENU_INTERCEPT_KEYS[index * 2 + 1].store(col, Ordering::Relaxed);
    }
}

/// Clear one intercept slot. The other slots are unaffected.
pub fn menu_intercept_clear_key(index: usize) {
    if index < MENU_SLOTS {
        MENU_INTERCEPT_KEYS[index * 2].store(SLOT_EMPTY, Ordering::Relaxed);
        MENU_INTERCEPT_KEYS[index * 2 + 1].store(SLOT_EMPTY, Ordering::Relaxed);
    }
}

/// Clear every intercept slot at once.
pub fn menu_intercept_clear_all() {
    for cell in MENU_INTERCEPT_KEYS.iter() {
        cell.store(SLOT_EMPTY, Ordering::Relaxed);
    }
}

/// `true` iff menu mode is active *and* `(row, col)` matches some slot.
#[inline]
pub fn should_intercept_key(row: u8, col: u8) -> bool {
    if !MENU_MODE_ACTIVE.load(Ordering::Relaxed) {
        return false;
    }
    for i in 0..MENU_SLOTS {
        let r = MENU_INTERCEPT_KEYS[i * 2].load(Ordering::Relaxed);
        let c = MENU_INTERCEPT_KEYS[i * 2 + 1].load(Ordering::Relaxed);
        if r == row && c == col {
            return true;
        }
    }
    false
}

/// `true` iff menu mode is active *and* encoder intercept is enabled.
#[inline]
pub fn should_intercept_encoder() -> bool {
    MENU_MODE_ACTIVE.load(Ordering::Relaxed) && MENU_INTERCEPT_ENCODER.load(Ordering::Relaxed)
}

// ---------------------------------------------------------------------------
// Deferred keys
// ---------------------------------------------------------------------------

const DEFERRED_SLOTS: usize = 8;

static DEFERRED_KEYS: [AtomicU8; DEFERRED_SLOTS * 2] = [const { AtomicU8::new(SLOT_EMPTY) }; DEFERRED_SLOTS * 2];

/// Configure slot `index` (0..8) so `(row, col)` becomes a deferred key.
/// Deferred keys are observed via the `KeyboardEvent` subscriber but their keymap
/// action is **never** dispatched to the host — the app decides what to send
/// (typically via [`send_keycode`] after running its own hold/tap timer).
pub fn deferred_key_set(index: usize, row: u8, col: u8) {
    if index < DEFERRED_SLOTS {
        DEFERRED_KEYS[index * 2].store(row, Ordering::Relaxed);
        DEFERRED_KEYS[index * 2 + 1].store(col, Ordering::Relaxed);
    }
}

/// Clear one deferred slot.
pub fn deferred_key_clear(index: usize) {
    if index < DEFERRED_SLOTS {
        DEFERRED_KEYS[index * 2].store(SLOT_EMPTY, Ordering::Relaxed);
        DEFERRED_KEYS[index * 2 + 1].store(SLOT_EMPTY, Ordering::Relaxed);
    }
}

/// `true` iff `(row, col)` matches some deferred slot.
#[inline]
pub fn is_deferred_key(row: u8, col: u8) -> bool {
    for i in 0..DEFERRED_SLOTS {
        let r = DEFERRED_KEYS[i * 2].load(Ordering::Relaxed);
        let c = DEFERRED_KEYS[i * 2 + 1].load(Ordering::Relaxed);
        if r == row && c == col {
            return true;
        }
    }
    false
}

/// `true` when the keyboard task should drop `event` without dispatching its
/// keymap action: a menu-intercepted key/encoder while in menu mode, or a deferred key.
#[inline]
pub(crate) fn should_swallow(event: &crate::event::KeyboardEvent) -> bool {
    match event.pos {
        crate::event::KeyboardEventPos::Key(pos) => {
            should_intercept_key(pos.row, pos.col) || is_deferred_key(pos.row, pos.col)
        }
        crate::event::KeyboardEventPos::RotaryEncoder(_) => should_intercept_encoder(),
        _ => false,
    }
}

// ---------------------------------------------------------------------------
// Manual key send (app → host)
// ---------------------------------------------------------------------------

/// Send a single keyboard report for `keycode` on the active transport:
/// `pressed = true` sends it as the only held key, `false` sends an all-up
/// report. Drops silently when no transport is active or its queue is full.
///
/// Consumer- and SystemControl-only keys (no HID equivalent via
/// `to_hid_keycode`) are ignored — this is intended for plain keyboard input.
pub fn send_keycode(keycode: KeyCode, pressed: bool) {
    use crate::hid::{KeyboardReport, Report};
    let mut report = KeyboardReport::default();
    if pressed {
        let hid_byte = match keycode {
            KeyCode::Hid(hk) => hk as u8,
            KeyCode::Consumer(ck) => match ck.to_hid_keycode() {
                Some(hk) => hk as u8,
                None => return,
            },
            KeyCode::SystemControl(sk) => match sk.to_hid_keycode() {
                Some(hk) => hk as u8,
                None => return,
            },
            // KeyCode is #[non_exhaustive] — future variants ignored.
            _ => return,
        };
        report.keycodes[0] = hid_byte;
    }
    crate::channel::try_send_hid_report(Report::KeyboardReport(report));
}

// ---------------------------------------------------------------------------
// Default layer / BLE profile
// ---------------------------------------------------------------------------

/// Pending default-layer switch requests, consumed by the keyboard task (which
/// owns the keymap) alongside its event wait.
pub(crate) static PENDING_DEFAULT_LAYER: Channel<RawMutex, u8, 4> = Channel::new();

/// Switch the default keymap layer at runtime. Silently drops if the queue
/// (depth 4) is already full.
pub fn set_default_layer(layer: u8) {
    let _ = PENDING_DEFAULT_LAYER.try_send(layer);
}

/// Switch to BLE profile `profile` (0-based). Drops silently when the profile
/// manager isn't consuming.
#[cfg(feature = "_ble")]
pub fn switch_ble_profile(profile: u8) {
    let _ = crate::channel::BLE_PROFILE_CHANNEL.try_send(crate::ble::profile::BleProfileAction::Switch(profile));
}

/// Clear the bonding info of the currently active BLE profile.
#[cfg(feature = "_ble")]
pub fn clear_ble_bond() {
    let _ = crate::channel::BLE_PROFILE_CHANNEL.try_send(crate::ble::profile::BleProfileAction::ClearBond);
}
