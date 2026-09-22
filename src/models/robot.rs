//! What a robot is to the simulator: a speed, a clock, and a position that advances.
//!
//! The database keeps `robots.capabilities` free-form so a new capability is a new key rather
//! than a migration. Two of those keys are not free-form to the simulator, though — it cannot
//! run a robot without knowing how fast it goes and how often it thinks — so [`RobotSpec`] is
//! where the untyped blob becomes the two numbers a run needs, once, at the point a run is
//! being set up rather than in its hot loop.

use crate::models::robots::unicycle_spec::{MAX_LINEAR_VEL, UnicycleSpec};
use crate::models::scale::METERS_PER_CELL;

/// Cells travelled per tick of the robot's own clock.
pub const MAX_VELOCITY: &str = "max_velocity";
/// Seconds between one move-and-replan and the next.
pub const TASK_INTERVAL: &str = "task_interval";

/// The capabilities a run reads, pulled out of `robots.capabilities`.
#[derive(Debug, Clone, Copy)]
pub struct RobotSpec {
    /// How far the robot gets per tick of its own clock, in cells *along its route*.
    ///
    /// **Derived, not stored.** The machine's real speed is
    /// [`body.max_linear_vel`](UnicycleSpec::max_linear_vel) in m/s; this is that speed
    /// expressed in the grid's units, via
    /// [`METERS_PER_CELL`](crate::models::scale::METERS_PER_CELL) and
    /// [`task_interval`](Self::task_interval). Keeping a stored `max_velocity` beside a stored
    /// `max_linear_vel` would be the same fact twice, free to disagree, with nothing to say
    /// which one the robot actually obeys.
    ///
    /// Route steps rather than Euclidean distance, so a diagonal counts as one the same as an
    /// orthogonal step does. That is a simplification and a visible one — a robot crossing a
    /// grid diagonally covers more ground per tick than one going along a row — but it keeps
    /// speed in the same units the route is written in, and the planner's own cost model
    /// (`ORTHOGONAL_COST` in movement_model.rs) is the place to make the two agree.
    pub max_velocity: f64,
    /// How often the robot moves and replans, in seconds. Independent of
    /// `grid_worlds.sim_interval`: the world's clock and the robot's are the experiment.
    pub task_interval: f64,
    /// The physical machine: its speed and acceleration limits, and how wide it is.
    ///
    /// The single source of truth for what the robot can do. A kinodynamic planner integrates
    /// it directly; the grid planners get [`max_velocity`](Self::max_velocity) derived from
    /// it, and clear obstacles by [`radius`](UnicycleSpec::radius).
    pub body: UnicycleSpec,
}

/// Why a robot's capabilities cannot drive a run.
#[derive(Debug)]
pub enum SpecError {
    /// The key is absent, or not a number.
    Missing(&'static str),
    /// Present and numeric, but not a speed or a period — zero, negative, or not finite.
    NotPositive(&'static str, f64),
    /// Present and numeric, but outside the range that key allows.
    ///
    /// Separate from [`NotPositive`](Self::NotPositive) because not every bound is "greater
    /// than zero": `min_linear_vel` is a reverse speed and has to be *non-positive*, and
    /// reporting that as "must be positive" would send the reader to fix the wrong sign.
    OutOfRange(&'static str, f64, &'static str),
}

impl std::fmt::Display for SpecError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SpecError::Missing(key) => {
                write!(f, "robot capabilities must include a numeric {key}")
            }
            SpecError::NotPositive(key, value) => {
                write!(f, "robot {key} must be a positive number, got {value}")
            }
            SpecError::OutOfRange(key, value, rule) => {
                write!(f, "robot {key} must be {rule}, got {value}")
            }
        }
    }
}

impl RobotSpec {
    /// Reads the two keys a run needs out of a robot's capabilities.
    ///
    /// Validated here as well as at the API boundary, because a row can predate a rule: these
    /// keys became required after robots already existed, and a run that trusted the blob
    /// would divide by a zero interval rather than say what is wrong with the robot.
    pub fn from_capabilities(capabilities: &serde_json::Value) -> Result<Self, SpecError> {
        let read = |key: &'static str| {
            let value = capabilities
                .get(key)
                .and_then(serde_json::Value::as_f64)
                .ok_or(SpecError::Missing(key))?;
            if !value.is_finite() || value <= 0.0 {
                return Err(SpecError::NotPositive(key, value));
            }
            Ok(value)
        };

        // Robots stored before the physical spec existed carry only `max_velocity`, in cells
        // per tick. Convert it *up* into a metric speed rather than carrying both: the rest of
        // the system then reads one number whatever the row's vintage, and the legacy key can
        // eventually be dropped without anything downstream noticing.
        //
        // Keyed on the absence of `max_linear_vel` rather than on a version flag, so a row
        // that gains the new key stops consulting the old one from that moment — the two can
        // never both be authoritative, which is the whole point.
        //
        // Read *before* `task_interval` even though the conversion below needs both, because
        // the order these are checked in is API-visible: capabilities missing everything has
        // always been reported against `max_velocity`, and clients read that message.
        let legacy_cells_per_tick = match capabilities.get(MAX_LINEAR_VEL) {
            Some(_) => None,
            None => Some(read(MAX_VELOCITY)?),
        };

        let task_interval = read(TASK_INTERVAL)?;
        let mut body = UnicycleSpec::from_capabilities(capabilities)?;
        if let Some(cells_per_tick) = legacy_cells_per_tick {
            body.max_linear_vel = cells_per_tick * METERS_PER_CELL / task_interval;
        }

        Ok(Self {
            max_velocity: body.max_linear_vel * task_interval / METERS_PER_CELL,
            task_interval,
            body,
        })
    }
}

/// Where a robot is, and how far along its route it has got.
///
/// Position is a whole cell because the world is: the robot is *at* a cell or it is not, and
/// a fractional position would have to be rounded before anything could plan from it. What is
/// fractional is the [`budget`](Self::budget) — a robot slower than one cell per tick banks
/// the remainder until it adds up to a step, which is what makes `max_velocity: 0.25` mean
/// "one cell every four ticks" rather than "no movement, ever".
#[derive(Debug, Clone)]
pub struct RobotBody {
    pub position: [i32; 2],
    pub dest: [i32; 2],
    /// The route as of the last replan, starting at [`position`](Self::position).
    pub route: Vec<(usize, usize)>,
    /// Distance earned but not yet spent, in cells. Always less than one after a step.
    budget: f64,
}

impl RobotBody {
    pub fn new(position: [i32; 2], dest: [i32; 2], route: Vec<(usize, usize)>) -> Self {
        Self {
            position,
            dest,
            route,
            budget: 0.0,
        }
    }

    /// Whether the robot is standing on its goal.
    pub fn arrived(&self) -> bool {
        self.position == self.dest
    }

    /// Walks up to `max_velocity` cells along the current route, and reports how many it
    /// managed.
    ///
    /// Stops at the end of the route rather than running past it: the route is the last thing
    /// the planner said was safe, and a robot that outran its plan would be moving through
    /// cells nothing has checked. A robot with no route — the goal is walled off this tick —
    /// stays put and banks nothing, so it does not lurch forward when the way reopens.
    ///
    /// `blocked` reports whether a cell is occupied *now*. The route was planned against the
    /// world as it stood at the last replan, and the obstacles have had ticks of their own
    /// since; without this the robot would walk into a shape that drifted across its path
    /// between one thought and the next. It stops on the last free cell rather than refusing
    /// to move at all, so it still makes what progress the route allows.
    pub fn advance(&mut self, max_velocity: f64, blocked: impl Fn([i32; 2]) -> bool) -> usize {
        if self.route.len() < 2 {
            self.budget = 0.0;
            return 0;
        }

        self.budget += max_velocity;
        // The route includes the cell the robot is standing on, so the steps available are
        // one fewer than its length.
        let available = self.route.len() - 1;
        let wanted = (self.budget.floor() as usize).min(available);
        if wanted == 0 {
            return 0;
        }

        // Walk cell by cell so the robot halts *at* the obstacle rather than tunnelling
        // through it: checking only the destination would let a fast robot step clean over a
        // shape standing in the middle of its path.
        let mut steps = 0;
        while steps < wanted {
            let (x, y) = self.route[steps + 1];
            if blocked([x as i32, y as i32]) {
                break;
            }
            steps += 1;
        }

        if steps == 0 {
            // Held up rather than idle. The unspent budget stays banked: the robot wanted to
            // move and was prevented, so it should not also lose the distance it had earned.
            return 0;
        }

        self.budget -= steps as f64;
        self.route.drain(..steps);
        let (x, y) = self.route[0];
        self.position = [x as i32, y as i32];
        steps
    }

    /// Replaces the route after a replan. The robot does not move; it just knows more.
    pub fn follow(&mut self, route: Vec<(usize, usize)>) {
        self.route = route;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn straight_line(len: usize) -> Vec<(usize, usize)> {
        (0..len).map(|x| (x, 0)).collect()
    }

    #[test]
    fn a_spec_needs_both_keys_to_be_positive_numbers() {
        let ok = serde_json::json!({"max_velocity": 1.5, "task_interval": 0.5});
        let spec = RobotSpec::from_capabilities(&ok).unwrap();
        assert_eq!(spec.max_velocity, 1.5);
        assert_eq!(spec.task_interval, 0.5);

        for bad in [
            serde_json::json!({"task_interval": 0.5}),
            serde_json::json!({"max_velocity": "fast", "task_interval": 0.5}),
            serde_json::json!({"max_velocity": 0, "task_interval": 0.5}),
            serde_json::json!({"max_velocity": -1, "task_interval": 0.5}),
            serde_json::json!({"max_velocity": 1}),
        ] {
            assert!(
                RobotSpec::from_capabilities(&bad).is_err(),
                "{bad} should not drive a run",
            );
        }
    }

    #[test]
    fn an_empty_blob_is_reported_against_max_velocity_first() {
        // The order the two required keys are checked in is API-visible — clients read the
        // message — so it is a contract, not an implementation detail. Pinned here as well as
        // in tests/test_routes.rs because reordering the reads is an easy, silent change and
        // an integration suite needing Postgres is a slow way to find out.
        let err = RobotSpec::from_capabilities(&serde_json::json!({})).unwrap_err();
        assert!(
            err.to_string().contains(MAX_VELOCITY),
            "an empty blob should name max_velocity, got {err}",
        );

        let err = RobotSpec::from_capabilities(&serde_json::json!({ "max_velocity": 1 }))
            .unwrap_err();
        assert!(
            err.to_string().contains(TASK_INTERVAL),
            "with a velocity present the next missing key is task_interval, got {err}",
        );
    }

    #[test]
    fn a_metric_speed_is_converted_into_cells_per_tick() {
        // The derivation that replaced a stored `max_velocity`. 2 m/s for half a second is
        // one metre, which at the current scale is one cell.
        let spec = RobotSpec::from_capabilities(&serde_json::json!({
            "max_velocity": 99,          // present but stale: must lose to the metric speed
            "task_interval": 0.5,
            "max_linear_vel": 2.0,
        }))
        .unwrap();

        assert_eq!(spec.body.max_linear_vel, 2.0);
        assert_eq!(
            spec.max_velocity,
            2.0 * 0.5 / METERS_PER_CELL,
            "the stale stored velocity won over the physical spec",
        );
    }

    #[test]
    fn a_robot_stored_before_the_physical_spec_still_runs() {
        // Every row in the fleet predates `max_linear_vel`, so the legacy path is not an edge
        // case — it is what the whole database takes today. The conversion has to be exact in
        // both directions, or upgrading the code would silently re-speed every stored robot.
        let legacy = RobotSpec::from_capabilities(&serde_json::json!({
            "max_velocity": 1.5,
            "task_interval": 0.5,
        }))
        .unwrap();

        assert_eq!(legacy.max_velocity, 1.5, "a stored velocity must survive the round trip");
        assert_eq!(
            legacy.body.max_linear_vel,
            1.5 * METERS_PER_CELL / 0.5,
            "cells per tick should have been read up into m/s",
        );
        // Everything the legacy row says nothing about falls back to the default machine.
        assert_eq!(legacy.body.track_width, UnicycleSpec::default().track_width);
    }

    #[test]
    fn the_two_velocities_can_never_disagree() {
        // The property the derivation exists for. Whichever key a row carries, the metric
        // speed and the cells-per-tick speed describe one machine — so converting either way
        // and back has to land where it started, for any plausible robot.
        for &(metric, interval) in &[(2.0, 0.5), (0.25, 2.0), (3.3, 0.1), (1.0, 1.0)] {
            let spec = RobotSpec::from_capabilities(&serde_json::json!({
                "task_interval": interval,
                "max_linear_vel": metric,
            }))
            .unwrap();
            let round_tripped = spec.max_velocity * METERS_PER_CELL / spec.task_interval;
            assert!(
                (round_tripped - metric).abs() < 1e-12,
                "{metric} m/s at {interval}s came back as {round_tripped}",
            );
        }
    }

    #[test]
    fn a_broken_physical_spec_is_reported_rather_than_defaulted() {
        // The physical keys are optional, but not arbitrary: a robot is refused at the API
        // boundary through this same function, so a nonsense track width has to surface here.
        assert!(
            RobotSpec::from_capabilities(&serde_json::json!({
                "max_velocity": 1, "task_interval": 0.5, "track_width": 0,
            }))
            .is_err(),
        );
    }

    #[test]
    fn a_whole_velocity_walks_that_many_cells() {
        let mut robot = RobotBody::new([0, 0], [4, 0], straight_line(5));
        assert_eq!(robot.advance(2.0, |_| false), 2);
        assert_eq!(robot.position, [2, 0]);
        assert_eq!(robot.advance(2.0, |_| false), 2);
        assert!(robot.arrived());
    }

    #[test]
    fn a_slow_robot_banks_its_remainder_until_it_buys_a_step() {
        // The property that makes fractional speeds mean anything: a quarter-speed robot
        // moves on every fourth tick rather than never.
        let mut robot = RobotBody::new([0, 0], [2, 0], straight_line(3));
        assert_eq!(robot.advance(0.25, |_| false), 0);
        assert_eq!(robot.advance(0.25, |_| false), 0);
        assert_eq!(robot.advance(0.25, |_| false), 0);
        assert_eq!(robot.position, [0, 0], "it should not have moved yet");
        assert_eq!(robot.advance(0.25, |_| false), 1);
        assert_eq!(robot.position, [1, 0]);
    }

    #[test]
    fn a_robot_never_outruns_its_route() {
        // The route is the last thing checked for obstacles, so overshooting it would put the
        // robot through cells nothing has looked at.
        let mut robot = RobotBody::new([0, 0], [9, 0], straight_line(3));
        assert_eq!(robot.advance(100.0, |_| false), 2);
        assert_eq!(robot.position, [2, 0]);
        assert!(!robot.arrived(), "it reached the route's end, not the goal");
    }

    #[test]
    fn a_robot_halts_at_an_obstacle_that_drifted_onto_its_route() {
        // The route was safe when it was planned; an obstacle has moved since. The robot must
        // stop on the last free cell rather than walk through the shape.
        let mut robot = RobotBody::new([0, 0], [4, 0], straight_line(5));
        assert_eq!(robot.advance(4.0, |cell| cell == [3, 0]), 2);
        assert_eq!(robot.position, [2, 0], "it should stop just short of the blocker");
        assert!(!robot.arrived());
    }

    #[test]
    fn a_fast_robot_cannot_tunnel_through_a_blocked_cell() {
        // Checking only the destination would let a robot with four cells of budget step
        // clean over a one-cell obstacle sitting in the middle of its path.
        let mut robot = RobotBody::new([0, 0], [4, 0], straight_line(5));
        assert_eq!(robot.advance(4.0, |cell| cell == [1, 0]), 0);
        assert_eq!(robot.position, [0, 0]);
    }

    #[test]
    fn a_blocked_robot_keeps_the_distance_it_earned() {
        // Being held up is not the same as standing still: the robot wanted to move, so the
        // budget it had banked should still be there when the way clears.
        let mut robot = RobotBody::new([0, 0], [2, 0], straight_line(3));
        assert_eq!(robot.advance(1.0, |cell| cell == [1, 0]), 0);
        assert_eq!(robot.advance(0.0, |_| false), 1, "the banked step should still buy a move");
        assert_eq!(robot.position, [1, 0]);
    }

    #[test]
    fn a_walled_off_robot_stays_put_without_banking() {
        // An empty route means the goal was unreachable this tick. Banking through the outage
        // would make the robot lurch several cells the moment the way reopened.
        let mut robot = RobotBody::new([0, 0], [4, 0], Vec::new());
        assert_eq!(robot.advance(1.0, |_| false), 0);
        assert_eq!(robot.advance(1.0, |_| false), 0);
        assert_eq!(robot.position, [0, 0]);

        robot.follow(straight_line(3));
        assert_eq!(robot.advance(1.0, |_| false), 1, "no banked distance from the outage");
    }
}
