//! The roughness of the country round a site.
//!
//! Undisturbed ground is not flat, and a lidar that sees a billiard table past the
//! last road learns nothing about what it will see in the field. So the ground away
//! from the site rolls a little: fractal value noise, seeded, so that one seed is
//! one landscape every time — the same property the rest of the generator has.

/// A 64-bit mix of three integers, well enough scrambled that neighbouring lattice
/// points are unrelated.
fn hash(x: i64, y: i64, seed: u64) -> u64 {
    let mut h = seed ^ 0x9e37_79b9_7f4a_7c15;
    for value in [x as u64, y as u64] {
        h ^= value
            .wrapping_add(0x9e37_79b9_7f4a_7c15)
            .wrapping_add(h << 6)
            .wrapping_add(h >> 2);
        h = h.wrapping_mul(0xbf58_476d_1ce4_e5b9);
        h ^= h >> 31;
    }
    h
}

/// A value in `[-1, 1)` at a lattice point.
fn lattice(x: i64, y: i64, seed: u64) -> f64 {
    (hash(x, y, seed) >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
}

/// Value noise: the lattice values blended smoothly across each cell.
fn value(x: f64, y: f64, seed: u64) -> f64 {
    let (i, j) = (x.floor(), y.floor());
    let (u, v) = (x - i, y - j);
    let (i, j) = (i as i64, j as i64);
    let fade = |t: f64| t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
    let (u, v) = (fade(u), fade(v));
    let a = lattice(i, j, seed) * (1.0 - u) + lattice(i + 1, j, seed) * u;
    let b = lattice(i, j + 1, seed) * (1.0 - u) + lattice(i + 1, j + 1, seed) * u;
    a * (1.0 - v) + b * v
}

/// Fractal noise in about `[-1, 1]`: four octaves, each half the wavelength and
/// half the amplitude of the one before, with `wavelength` metres the longest.
pub fn fractal(x: f64, y: f64, wavelength: f64, seed: u64) -> f64 {
    let mut total = 0.0;
    let mut amplitude = 1.0;
    let mut frequency = 1.0 / wavelength.max(1e-3);
    let mut norm = 0.0;
    for octave in 0..4u64 {
        total += amplitude * value(x * frequency, y * frequency, seed.wrapping_add(octave));
        norm += amplitude;
        amplitude *= 0.5;
        frequency *= 2.0;
    }
    total / norm
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_seed_is_one_landscape() {
        assert_eq!(
            fractal(12.5, -40.0, 100.0, 7),
            fractal(12.5, -40.0, 100.0, 7)
        );
        assert_ne!(
            fractal(12.5, -40.0, 100.0, 7),
            fractal(12.5, -40.0, 100.0, 8)
        );
        for step in 0..200 {
            let value = fractal(step as f64 * 3.7, step as f64 * -1.3, 50.0, 1);
            assert!((-1.0..=1.0).contains(&value));
        }
    }
}
