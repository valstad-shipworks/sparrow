#![cfg_attr(feature = "simd", feature(portable_simd))]
#![allow(const_item_mutation)]
#![allow(unused_imports)]

use jagua_rs::probs::spp::io::ext_repr::{ExtSPInstance, ExtSPSolution};

pub use jagua_rs;
pub mod config;
pub mod consts;
pub mod eval;
pub mod optimizer;
pub mod quantify;
pub mod sample;
pub mod util;

pub use config::*;
pub use optimizer::optimize;

#[derive(Clone)]
pub struct SPOutput {
    pub instance: ExtSPInstance,
    pub solution: ExtSPSolution,
}