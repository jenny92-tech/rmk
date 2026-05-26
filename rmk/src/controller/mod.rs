//! Runtime controller API for companion apps (k9pad-style).
//!
//! Lets a firmware app:
//! - subscribe to live key events ([`key_event_subscriber`])
//! - configure a small set of *menu intercept* slots that swallow press/release
//!   without delivering them to the host while the app is in menu mode
//! - configure a small set of *deferred* slots: the firmware suppresses the
//!   normal press/release on the host but still publishes the [`KeyEvent`] so
//!   the app can decide whether to forward, drop, or transform it
//! - inject manual keystrokes onto the active transport via [`send_keycode`]
//!
//! Everything in this module is gated by the `controller` feature. The state
//! lives in static atomics so there is no per-instance bookkeeping — the
//! firmware app owns the API surface as plain free functions.
//!
//! Hold-tap timing is **not** provided here: the app builds its own using
//! [`embassy_time::Timer`] racing the subscriber, then calls
//! [`send_keycode`] to fire the tap action on short-release.

use core::sync::atomic::{AtomicBool, AtomicU8, Ordering};

use embassy_sync::channel::Channel;
use embassy_sync::pubsub::{PubSubChannel, Subscriber};
use rmk_types::action::KeyAction;
use rmk_types::keycode::KeyCode;

use crate::RawMutex;
use crate::event::KeyboardEvent;

// ---------------------------------------------------------------------------
// Key event pub/sub
// ---------------------------------------------------------------------------

/// Live key event published to controller subscribers each time the
/// keyboard scans a configured deferred or menu-intercept key.
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KeyEvent {
    pub keyboard_event: KeyboardEvent,
    pub key_action: KeyAction,
}

// One publisher (the keyboard task) and up to two subscribers (e.g. UI +
// background telemetry). Queue depth 4 absorbs short bursts.
static KEY_EVENT_CHANNEL: PubSubChannel<RawMutex, KeyEvent, 4, 2, 1> = PubSubChannel::new();

/// Subscribe to firmware key events. Returns `None` if the subscriber cap
/// (currently 2) is already full.
pub fn key_event_subscriber() -> Option<Subscriber<'static, RawMutex, KeyEvent, 4, 2, 1>> {
    KEY_EVENT_CHANNEL.subscriber().ok()
}

#[inline]
pub(crate) fn publish_key_event(event: KeyEvent) {
    KEY_EVENT_CHANNEL.immediate_publisher().publish_immediate(event);
}

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
/// Deferred keys are observed via [`key_event_subscriber`] but their keymap
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

// ---------------------------------------------------------------------------
// Manual key send (app → host)
// ---------------------------------------------------------------------------

/// Queue depth — drops are silent (we never want to block the caller).
const PENDING_KEYCODE_DEPTH: usize = 8;

pub(crate) static PENDING_KEYCODES: Channel<RawMutex, (KeyCode, bool), PENDING_KEYCODE_DEPTH> = Channel::new();

/// Enqueue a single (keycode, pressed) pair to be sent on the active
/// transport at the next keyboard tick. Silently drops if the queue is full;
/// callers that need delivery guarantees should rate-limit themselves.
pub fn send_keycode(keycode: KeyCode, pressed: bool) {
    let _ = PENDING_KEYCODES.try_send((keycode, pressed));
}

/// Drain `PENDING_KEYCODES` and emit each as a KeyboardReport on the active
/// transport. Called by the keyboard main loop every tick.
///
/// Consumer- and SystemControl-only keys (no HID equivalent via
/// `to_hid_keycode`) are silently dropped here — `send_keycode` is intended
/// for plain keyboard input. Use the dedicated media/system report channels
/// for those usage pages.
pub(crate) fn process_pending_keycodes() {
    use crate::hid::{KeyboardReport, Report};
    while let Ok((keycode, pressed)) = PENDING_KEYCODES.try_receive() {
        let mut report = KeyboardReport::default();
        if pressed {
            let hid_byte = match keycode {
                KeyCode::Hid(hk) => hk as u8,
                KeyCode::Consumer(ck) => match ck.to_hid_keycode() {
                    Some(hk) => hk as u8,
                    None => continue,
                },
                KeyCode::SystemControl(sk) => match sk.to_hid_keycode() {
                    Some(hk) => hk as u8,
                    None => continue,
                },
                // KeyCode is #[non_exhaustive] — future variants ignored.
                _ => continue,
            };
            report.keycodes[0] = hid_byte;
        }
        crate::channel::try_send_hid_report(Report::KeyboardReport(report));
    }
}
