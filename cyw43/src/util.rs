#![allow(unused)]

use core::{mem, ptr, slice};

use aligned::{A4, Aligned};
use embassy_time::{Duration, Ticker};

use crate::WithContext;

/// Defines a `repr(u8)` enum and implements a `from()` associated function to instantiate it from
/// a `u8`, defaulting to the variant decorated with `#[default]`.
macro_rules! enum_from_u8 {
    (
        $( #[$enum_attr:meta] )*
        enum $enum:ident {
            // NOTE: The default variant must be the first variant.
            // Additionally, the `#[default]` attribute must be placed before any other attributes
            // on the variant, to avoid a parsing ambiguity.
            #[default]
            $( #[$default_variant_attr:meta] )*
            $default_variant:ident = $default_value:literal,
            $(
                $( #[$variant_attr:meta] )*
                $variant:ident = $value:literal
            ),+
            $(,)?
        }
    ) => {
        $( #[$enum_attr] )*
        #[repr(u8)]
        pub enum $enum {
            $( #[$default_variant_attr] )*
            $default_variant = $default_value,
            $(
                $( #[$variant_attr] )*
                $variant = $value
            ),+
        }

        impl $enum {
            pub fn from(value: u8) -> Self {
                match value {
                    $default_value => Self::$default_variant,
                    $( $value => Self::$variant ),+,
                    _ => Self::$default_variant,
                }
            }
        }
    };
}
pub(crate) use enum_from_u8;

pub(crate) fn is_aligned(a: u32, x: u32) -> bool {
    (a & (x - 1)) == 0
}

pub(crate) fn round_down(x: u32, a: u32) -> u32 {
    debug_assert!(a.is_power_of_two());

    x & !(a - 1)
}

pub(crate) fn round_up(x: u32, a: u32) -> u32 {
    debug_assert!(a.is_power_of_two());

    (x + (a - 1)) & !(a - 1)
}

pub(crate) async fn try_until(mut func: impl AsyncFnMut() -> bool, duration: Duration) -> crate::Result<()> {
    let tick = Duration::from_millis(1);
    let mut ticker = Ticker::every(tick);
    let ticks = duration.as_ticks() / tick.as_ticks();

    for _ in 0..ticks {
        if func().await {
            return Ok(());
        }

        ticker.next().await;
    }

    Err(crate::Error)
}

/// Log the first few of a repeating condition, then one in `EVERY` after that.
///
/// When the shared bus goes wrong it goes wrong on every poll, and an
/// unthrottled `warn!` per poll buries the transition — the only part of the
/// log that carries any information — under a quarter of a million identical
/// lines.
pub(crate) struct Throttle {
    seen: u32,
    every: u32,
}

impl Throttle {
    const FIRST: u32 = 8;

    pub(crate) const fn new() -> Self {
        Self::every(4096)
    }

    /// A throttle that reports one in `every` after the first few, for a signal
    /// rare enough that 4096 would hide whether it is still happening.
    pub(crate) const fn every(every: u32) -> Self {
        Self { seen: 0, every }
    }

    /// Whether this occurrence should be logged, and how many have been seen.
    pub(crate) fn admit(&mut self) -> Option<u32> {
        self.seen += 1;
        (self.seen <= Self::FIRST || self.seen % self.every == 0).then_some(self.seen)
    }
}
