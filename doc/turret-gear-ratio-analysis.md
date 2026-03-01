# Turret Gear Ratio Analysis

## Current Configuration

- **Servo:** Axon Max CRServo (×2, both receive identical power)
- **Gear ratio:** 1:1 (one servo rotation = one turret rotation)
- **Axon Max free speed:** 60° / 0.115 s = **522°/s** at 6V
- **Operating voltage:** 6V
- **Encoder:** Absolute analog (0–3.3V → 0–360°)

## The Problem: kS Dominates the Control Range

| Parameter | Value | Notes |
|-----------|-------|-------|
| kV (velocity FF) | 0.00068 | Power per °/s |
| kS_CW (stiction, positive power) | 0.084 | Dead-zone compensation |
| kS_CCW (stiction, negative power) | 0.072 | |
| maxProfileVelocity | 120°/s | Cruise speed target |
| maxOutputPower | 0.35 | Safety clamp |
| Cruise FF (kV·120) | 0.082 | Velocity-proportional term |
| Cruise FF + 30% kS | ~0.107 | With kinetic friction comp |

**kS is ~75% of cruise feedforward.** The stiction compensation (0.08) is nearly as
large as the velocity term (0.082). This means:

1. **The transition from stopped → moving is a massive step** — power jumps from 0
   to 0.08 instantly just to overcome friction, before any velocity-proportional
   drive begins. This discrete jump is the root cause of power jitter at low speeds.

2. **At cruise, the servo operates at ~11% of its power range** (0.107 out of 1.0).
   A 0.01 PID correction — which looks tiny — represents a ~10% change in cruise
   power and produces ~5–6°/s velocity change. The controller is working in a
   narrow band where encoder noise creates proportionally large disturbances.

3. **The kS fade is critical and hard to tune** because kS is such a large fraction
   of total power. Every iteration of the fade approach (actual-velocity ratio,
   filtered output, filtered velocity input) ran into either jitter (from encoder
   noise amplified through the kS term) or phase-lag oscillation (from filtering
   that noise).

## How Gearing Would Help

With a gear reduction ratio $N$:1 (servo:turret):

- **Max turret speed** = 522 / $N$ °/s
- **Cruise power** scales up roughly by $N$ (ignoring friction changes)
- **kS stays approximately constant** (friction is in the drivetrain, not speed-dependent)
- **kS as a fraction of cruise power** drops by ~$1/N$

| Ratio | Max Speed | Cruise Power (est.) | kS / Cruise | Control Resolution |
|-------|-----------|--------------------:|:-----------:|:------------------:|
| 1:1 | 522°/s | ~0.107 | **75%** | 0.01 → ~5°/s |
| 2:1 | 261°/s | ~0.19 | 42% | 0.01 → ~2.7°/s |
| 3:1 | 174°/s | ~0.28 | 29% | 0.01 → ~1.7°/s |
| 5:1 | 104°/s | ~0.46 | 17% | 0.01 → ~1.0°/s |

> **Note:** Max speed at 5:1 (104°/s) would be too slow for the current 120°/s
> cruise target. A 3:1 ratio at 174°/s max would support 120°/s cruise with
> ~30% headroom, which is a reasonable operating point.

### At 3:1 Reduction

- Cruise power rises to ~0.28 → well above the noise floor
- kS (0.08) drops to 29% of cruise → the stiction transition is proportionally smaller
- A 0.01 PID correction = 3.6% of cruise power → much less disruptive
- The kS fade becomes less critical — even a simple on/off threshold works
  acceptably when kS is only 29% of the signal
- **The controller would likely work with just P + kV (no kS fade, no output
  filter, no velocity smoothing)**

## Why the 1:1 Ratio Makes Tuning Hard

Every tuning issue encountered traces back to kS being 75% of cruise power:

| Issue | Root Cause | Would 3:1 Fix It? |
|-------|-----------|:------------------:|
| Power jitter at cruise | kS fade reacts to encoder noise; kS is huge | Yes — kS is 29%, noise effect ÷3 |
| Overshoot from kS at cruise | Full kS overdrives at cruise speed | Yes — 29% kS doesn't overdrive |
| Output filter oscillation | Filter delays kS fade → positive feedback | Yes — no kS fade needed |
| Velocity smoothing oscillation | Smoothing delays kS fade → same problem | Yes — no kS fade needed |
| Velocity oscillation without kS | kV alone has kinetic friction deficit, PID chases | Partially — deficit is smaller fraction |
| CW/CCW asymmetry | kS_CW ≠ kS_CCW, and kS is 75% of cruise | Yes — asymmetry becomes 29% × 17% = ~5% effect |

## Conclusion

The 1:1 gear ratio forces the controller to operate in a regime where stiction
compensation dominates the drive signal. This makes the stiction-to-moving
transition disproportionately large, amplifies encoder noise into visible
velocity disturbances, and requires sophisticated fade/filter strategies that
each introduce their own failure modes.

A 3:1 reduction would shift cruise power into the 25-30% range where kS becomes
a minor correction rather than the primary drive component. The controller would
be dramatically simpler to tune — likely P + kV with no fade logic needed.

**However**, the 1:1 ratio is a hardware constraint that cannot be changed without
a redesign. The current approach (desired-velocity-based kS fade + PID-only
output filter) is the best software mitigation available and is producing
acceptable results.
