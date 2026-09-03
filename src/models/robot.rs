//! What a robot is to the simulator: a speed, a clock, and a position that advances.
//!
//! The database keeps `robots.capabilities` free-form so a new capability is a new key rather
//! than a migration. Two of those keys are not free-form to the simulator, though — it cannot
//! run a robot without knowing how fast it goes and how often it thinks — so [`RobotSpec`] is
//! where the untyped blob becomes the two numbers a run needs, once, at the point a run is
//! being set up rather than in its hot loop.

use serde::Deserialize;

/// Cells travelled per tick of the robot's own clock.
pub const MAX_VELOCITY: &str = "max_velocity";
/// Seconds between one move-and-replan and the next.
pub const TASK_INTERVAL: &str = "task_interval";

/// The capabilities a run reads, pulled out of `robots.capabilities`.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct RobotSpec {
    /// How far the robot gets per tick of its own clock, in cells *along its route*.
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
}

/// Why a robot's capabilities cannot drive a run.
#[derive(Debug)]
pub enum SpecError {
    /// The key is absent, or not a number.
    Missing(&'static str),
    /// Present and numeric, but not a speed or a period — zero, negative, or not finite.
    NotPositive(&'static str, f64),
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

        Ok(Self {
            max_velocity: read(MAX_VELOCITY)?,
            task_interval: read(TASK_INTERVAL)?,
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
    pub fn advance(&mut self, max_velocity: f64) -> usize {
        if self.route.len() < 2 {
            self.budget = 0.0;
            return 0;
        }

        self.budget += max_velocity;
        // The route includes the cell the robot is standing on, so the steps available are
        // one fewer than its length.
        let available = self.route.len() - 1;
        let steps = (self.budget.floor() as usize).min(available);
        if steps == 0 {
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
    fn a_whole_velocity_walks_that_many_cells() {
        let mut robot = RobotBody::new([0, 0], [4, 0], straight_line(5));
        assert_eq!(robot.advance(2.0), 2);
        assert_eq!(robot.position, [2, 0]);
        assert_eq!(robot.advance(2.0), 2);
        assert!(robot.arrived());
    }

    #[test]
    fn a_slow_robot_banks_its_remainder_until_it_buys_a_step() {
        // The property that makes fractional speeds mean anything: a quarter-speed robot
        // moves on every fourth tick rather than never.
        let mut robot = RobotBody::new([0, 0], [2, 0], straight_line(3));
        assert_eq!(robot.advance(0.25), 0);
        assert_eq!(robot.advance(0.25), 0);
        assert_eq!(robot.advance(0.25), 0);
        assert_eq!(robot.position, [0, 0], "it should not have moved yet");
        assert_eq!(robot.advance(0.25), 1);
        assert_eq!(robot.position, [1, 0]);
    }

    #[test]
    fn a_robot_never_outruns_its_route() {
        // The route is the last thing checked for obstacles, so overshooting it would put the
        // robot through cells nothing has looked at.
        let mut robot = RobotBody::new([0, 0], [9, 0], straight_line(3));
        assert_eq!(robot.advance(100.0), 2);
        assert_eq!(robot.position, [2, 0]);
        assert!(!robot.arrived(), "it reached the route's end, not the goal");
    }

    #[test]
    fn a_walled_off_robot_stays_put_without_banking() {
        // An empty route means the goal was unreachable this tick. Banking through the outage
        // would make the robot lurch several cells the moment the way reopened.
        let mut robot = RobotBody::new([0, 0], [4, 0], Vec::new());
        assert_eq!(robot.advance(1.0), 0);
        assert_eq!(robot.advance(1.0), 0);
        assert_eq!(robot.position, [0, 0]);

        robot.follow(straight_line(3));
        assert_eq!(robot.advance(1.0), 1, "no banked distance from the outage");
    }
}
