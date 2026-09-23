//! The kinodynamic planning stack: a robot model, an integrator, and SST over them.
//!
//! `allow(dead_code)` because none of this is reachable from a running server yet — it is
//! exercised only by its own tests. Wiring it in means a `PlannerKind` variant and a decision
//! about the wire format, since a trajectory is a sequence of continuous poses where every
//! existing planner returns a list of cells. The allow comes off with that change.
#![allow(dead_code)]

pub(crate) mod grid_adapter;
pub(crate) mod kino_dynamic_edge_generator;
pub(crate) mod planning_world;
pub(crate) mod sst;
pub(crate) mod trace;
pub(crate) mod trajector_edge;
