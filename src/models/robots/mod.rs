//! The kinodynamic planning stack: a robot model, an integrator, and SST over them.
//!
//! `allow(dead_code)` because none of this is reachable from a running server yet — it is
//! exercised only by its own tests. Wiring it in means a `PlannerKind` variant and a decision
//! about the wire format, since a trajectory is a sequence of continuous poses where every
//! existing planner returns a list of cells. The allow comes off with that change.
#![allow(dead_code)]

pub(crate) mod unicycle_spec;
pub(crate) mod unicycle_state;