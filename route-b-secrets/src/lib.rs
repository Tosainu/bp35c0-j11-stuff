#![no_std]

#[cfg(not(route_b_fallback_keys))]
#[rustfmt::skip]
mod secrets;

#[cfg(route_b_fallback_keys)]
mod secrets {
    pub const ROUTE_B_ID: &[u8; 32] = b"xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx";
    pub const ROUTE_B_PASSWORD: &[u8; 12] = b"xxxxxxxxxxxx";
}

pub const ROUTE_B_ID: &[u8; 32] = secrets::ROUTE_B_ID;
pub const ROUTE_B_PASSWORD: &[u8; 12] = secrets::ROUTE_B_PASSWORD;
