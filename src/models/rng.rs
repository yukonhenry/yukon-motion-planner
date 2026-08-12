//! A tiny deterministic pseudo-random generator.
//!
//! Deliberately not the `rand` crate: the only caller is obstacle jitter, which needs a
//! *reproducible* sequence far more than a statistically excellent one. A simulation that
//! wanders somewhere interesting has to be replayable from its seed, and a run that exposes a
//! planner bug has to be reproducible from the same. Xorshift64 is a few lines, has no
//! dependency, and gives the same stream on every platform and every build.
//!
//! Not suitable for anything where predictability matters — no keys, no tokens, no sampling
//! that must resist an adversary.

/// Marsaglia's xorshift64, as a plain counter of a state.
#[derive(Debug, Clone)]
pub struct Xorshift {
    state: u64,
}

impl Xorshift {
    /// Seeds the generator. Zero is remapped, because xorshift's one fixed point is 0 — seeded
    /// there it returns 0 forever, which would silently mean "nothing ever moves".
    pub fn new(seed: u64) -> Self {
        Self {
            state: if seed == 0 {
                0x9E37_79B9_7F4A_7C15
            } else {
                seed
            },
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        self.state
    }

    /// A value in `0..bound`. Panics on `bound == 0`, which has no answer to give.
    ///
    /// Modulo, so the lowest `bound` values are very slightly favored when `bound` does not
    /// divide 2^64. At the bounds used here — vertex counts and 8 directions — the bias is far
    /// below anything a jittering obstacle could express.
    pub fn below(&mut self, bound: usize) -> usize {
        assert!(bound > 0, "no value is below 0");
        (self.next_u64() % bound as u64) as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_same_seed_gives_the_same_stream() {
        // The whole reason this exists rather than `rand`.
        let mut a = Xorshift::new(0x2026_0811);
        let mut b = Xorshift::new(0x2026_0811);
        let left: Vec<u64> = (0..16).map(|_| a.next_u64()).collect();
        let right: Vec<u64> = (0..16).map(|_| b.next_u64()).collect();
        assert_eq!(left, right);
    }

    #[test]
    fn different_seeds_diverge() {
        let mut a = Xorshift::new(1);
        let mut b = Xorshift::new(2);
        assert_ne!(a.next_u64(), b.next_u64());
    }

    #[test]
    fn a_zero_seed_still_moves() {
        // Xorshift's fixed point. Seeded at 0 without the remap this yields 0 forever, and an
        // obstacle simulation would look like it was working while standing perfectly still.
        let mut rng = Xorshift::new(0);
        assert!(
            (0..8).any(|_| rng.next_u64() != 0),
            "seed 0 froze the stream"
        );
    }

    #[test]
    fn below_stays_in_range_and_reaches_both_ends() {
        let mut rng = Xorshift::new(7);
        let draws: Vec<usize> = (0..600).map(|_| rng.below(8)).collect();
        assert!(draws.iter().all(|&d| d < 8), "out of range");
        assert!(
            draws.contains(&0) && draws.contains(&7),
            "never reached an end"
        );
    }

    #[test]
    fn below_one_is_always_zero() {
        // The single-choice case, which the vertex picker hits on a degenerate polygon.
        let mut rng = Xorshift::new(99);
        assert!((0..10).all(|_| rng.below(1) == 0));
    }
}
