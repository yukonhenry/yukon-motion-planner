// Using 3D for spatial tracking

use crate::models::robots::unicycle_state::{UnicycleControl, UnicycleState};

/// Represents a dynamically feasible path segment connecting two nodes in the RRT tree
#[derive(Clone)]
pub(crate) struct TrajectoryEdge {
    pub states: Vec<UnicycleState>,    // Fine-grained path history along the edge
    pub controls: Vec<UnicycleControl>, // Control inputs applied at each dt step
    pub cost: f64,                     // Total duration or distance of this edge
}
