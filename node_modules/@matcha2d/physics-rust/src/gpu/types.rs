//! GPU buffer layouts — must match `shaders/physics.wgsl`.

use bytemuck::{Pod, Zeroable};

pub const GPU_MAX_BODIES: usize = 2048;
pub const GPU_MAX_COLLIDERS: usize = 2048;
pub const GPU_MAX_PAIRS: usize = 8192;
pub const GPU_MAX_POLY_VERTS: usize = 8;
/// Flat `x,y` pairs per collider slot.
pub const GPU_POLY_BUFFER_FLOATS: usize = GPU_MAX_COLLIDERS * GPU_MAX_POLY_VERTS * 2;

/// Uniform simulation parameters (padded to 256 bytes for WebGPU uniform rules).
#[repr(C, align(16))]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct GpuSimParams {
    pub dt: f32,
    pub gravity_x: f32,
    pub gravity_y: f32,
    pub pair_count: u32,
    pub body_count: u32,
    pub collider_count: u32,
    pub solver_iterations: u32,
    pub substeps: u32,
    pub penetration_slop: f32,
    /// Clamps per-iteration normal position correction (matches CPU Baumgarte velocity cap scale: `≤ max * dt`).
    pub max_corrective_velocity: f32,
    pub _pad1: f32,
    pub _pad2: f32,
    /// Zero padding to 256-byte uniform size (`48 + 26×8`).
    pub _uniform_tail: [u64; 26],
}

impl GpuSimParams {
    pub fn padded_bytes() -> usize {
        std::mem::size_of::<Self>()
    }
}

#[repr(C, align(16))]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct GpuBody {
    pub position: [f32; 2],
    pub angle: f32,
    pub angvel: f32,
    pub linvel: [f32; 2],
    pub inv_mass: f32,
    pub inv_inertia: f32,
    pub body_type: u32,
    pub flags: u32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
    pub _pad_b: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct GpuCollider {
    pub shape_kind: u32,
    pub body_index: u32,
    pub local_pos: [f32; 2],
    pub local_sin: f32,
    pub local_cos: f32,
    pub friction: f32,
    pub restitution: f32,
    pub collider_flags: u32,
    pub param0: f32,
    pub param1: f32,
    pub poly_vert_start: u32,
    pub poly_vert_count: u32,
    pub _p0: u32,
    pub _p1: u32,
    pub _p2: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct GpuContactPoint {
    pub local_a: [f32; 2],
    pub local_b: [f32; 2],
    pub penetration: f32,
    pub _pad: f32,
}

/// Must match WGSL `Manifold` including padding before `c0`.
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct GpuManifold {
    pub valid: u32,
    pub collider_a: u32,
    pub collider_b: u32,
    pub sensor_pair: u32,
    pub normal: [f32; 2],
    pub tangent: [f32; 2],
    pub penetration: f32,
    pub contact_count: u32,
    /// `dist` = dist_offset + dot(world_b - world_a, normal); penetration = max(0, -dist)
    pub dist_offset: f32,
    pub _man_pad1: u32,
    pub _man_pad2: u32,
    pub _man_pad3: u32,
    pub _man_pad4: u32,
    pub _man_pad5: u32,
    pub c0: GpuContactPoint,
    pub c1: GpuContactPoint,
    /// Pads struct to 128 bytes for WGSL `array<Manifold>` stride.
    pub _stride_pad: [u32; 4],
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gpu_layout_sizes() {
        assert_eq!(std::mem::size_of::<GpuBody>(), 64);
        assert_eq!(std::mem::size_of::<GpuCollider>(), 64);
        assert_eq!(std::mem::size_of::<GpuManifold>(), 128);
        assert_eq!(std::mem::size_of::<GpuSimParams>(), 256);
    }
}
