# Turret Gear Ratio Analysis

## Current Configuration (Iteration 29 — Final)

- **Servo:** Axon Max CRServo (×2, both receive identical power)
- **Gear ratio:** 1:1 (one servo rotation = one turret rotation)
- **Axon Max free speed:** 60° / 0.115 s = **522°/s** at 6V
- **Operating voltage:** 6V
- **Encoder:** Absolute analog (0–3.3V → 0–360°)
- **maxProfileVelocity:** 400°/s (actual cruise: 250–280°/s)
- **maxOutputPower:** 0.50

## The Original Problem: kS Dominated at Low Speed

Early in tuning (iterations 1–16), the cruise target was 120°/s:

| Parameter | Value (iter ≤16) | Notes |
|-----------|-----------------|-------|
| kV (velocity FF) | 0.00068 | Power per °/s |
| kS_CW (stiction) | 0.084 | Dead-zone compensation |
| kS_CCW (stiction) | 0.072 | |
| maxProfileVelocity | 120°/s | Cruise speed target |
| maxOutputPower | 0.35 | Safety clamp |
| Cruise FF (kV·120) | 0.082 | Velocity-proportional term |
| kS / Cruise FF | **~100%** | kS ≈ kV·vel at this speed |

**kS was ~75–100% of cruise feedforward.** The stiction compensation (0.08) was
nearly as large as the velocity term (0.082). This caused:

1. **A massive step from stopped → moving** — power jumped from 0 to 0.08 to
   overcome friction, before any velocity drive began.
2. **Narrow operating band** — cruise at ~0.107 (11% of power range), so a 0.01
   PID correction was a ~10% change in cruise power.
3. **kS fade was critical and hard to tune** — every approach (actual-velocity
   ratio, filtered output, filtered velocity input) had failure modes because
   kS was such a large fraction of total power.

## How Higher Speed Changed the Picture

By pushing cruise speed from 120°/s to 280°/s (iterations 17–29), the kV·vel
term grew to dominate over kS:

| Operating Point | kV·vel | kS_CW | kS / kV·vel | Total Cruise | Cruise / Max Power |
|-----------------|--------|-------|:-----------:|:------------:|:------------------:|
| 120°/s (iter ≤16) | 0.082 | 0.084 | **102%** | ~0.107 | 11% of 1.0 |
| 280°/s (iter 29) | 0.190 | 0.084 | **44%** | ~0.201 | 40% of 0.50 |

At 280°/s, kV·vel = 0.190 — more than double the stiction term. Additionally,
the **desired-velocity kS fade** (kSVelRatioZero=1.15) reduces kS at cruise:

- At cruise: velRatio ≈ 1.0, kSScale = (1.15 − 1.0) / 1.15 = **0.13**
- Effective kS at cruise = 0.084 × 0.13 = **0.011**
- kS is now only **5.5% of cruise power** (was 75–100%)
- A 0.01 PID correction = 5% of cruise power (was ~10%)

The combination of higher speed + kS fade moved the controller from a
kS-dominated regime into a kV-dominated regime — even at 1:1 gearing.

## How Gearing Would Help (Theoretical)

With a gear reduction ratio $N$:1 (servo:turret):

- **Max turret speed** = 522 / $N$ °/s
- **Cruise power** scales up roughly by $N$ (ignoring friction changes)
- **kS stays approximately constant** (friction is in the drivetrain, not speed-dependent)
- **kS as a fraction of cruise power** drops by ~$1/N$

| Ratio | Max Speed | Cruise Power (est.) | Raw kS / Cruise | Effective kS (w/ fade) | Control Resolution |
|-------|-----------|--------------------:|:-----------:|:----------------------:|:------------------:|
| **1:1 (actual)** | 522°/s | ~0.201 @ 280°/s | 44% | **5.5%** | 0.01 → ~2.6°/s |
| 2:1 | 261°/s | ~0.38 @ 280°/s | 22% | — | 0.01 → ~1.4°/s |
| 3:1 | 174°/s | — | — | — | — |

> **Note:** At 3:1 and above, max turret speed drops below our 280°/s cruise
> target — those ratios are no longer viable at the iteration 29 speed.
> A 2:1 ratio (261°/s max) would barely support our actual cruise of 250–280°/s
> with negligible headroom.

### Comparison: 1:1 at 280°/s vs. 3:1 at 120°/s

The original motivation for gearing was to bring kS below ~30% of cruise power.
Higher speed on 1:1 achieved a similar effect through a different path:

| Metric | 1:1 @ 120°/s | 3:1 @ 120°/s | 1:1 @ 280°/s |
|--------|:------------:|:------------:|:-------------:|
| Raw kS / Cruise | 102% | 29% | 44% |
| Effective kS (w/ fade) | N/A | ~29% | **5.5%** |
| Cruise / Max Power | 11% | 28% | **40%** |
| 0.01 PID = % of cruise | 9.3% | 3.6% | **5.0%** |
| Software complexity | — | Simple P+kV | PID + kS fade + filters |

At 280°/s with the kS fade, the 1:1 ratio actually achieves *better* effective
kS suppression (5.5%) than 3:1 gearing without a fade (29%). The tradeoff is
significantly more software complexity to get there.

## Why the 1:1 Ratio Made Tuning Hard

Every tuning issue in iterations 1–16 traced back to kS being 75–100% of cruise
power at 120°/s:

| Issue | Root Cause | Resolved? | How |
|-------|-----------|:---------:|-----|
| Power jitter at cruise | kS fade reacts to encoder noise | ✅ | Higher speed → kS fade is gentler |
| Overshoot from kS at cruise | Full kS overdrives at cruise speed | ✅ | kS faded to 5.5% at cruise |
| Output filter oscillation | Filter delays kS fade → positive feedback | ✅ | PID-only output filter (iter 13) |
| Velocity smoothing oscillation | Smoothing delays kS fade → same problem | ✅ | Desired-velocity kS fade (iter 16) |
| Velocity oscillation without kS | kV alone has kinetic friction deficit | ✅ | kS fade maintains small residual |
| CW/CCW asymmetry | kS_CW ≠ kS_CCW, 75% of cruise | ✅ | kSVelRatioZero=1.15 + higher speed |
| Short-move overshoot | Accel too aggressive for 20° distance | ✅ | Distance-dependent accel scaling (iter 29) |

All issues were resolved through software — no hardware changes required.

## Software Mitigations That Made 1:1 Work

The following techniques, developed over 29 iterations, compensate for the
challenges of the 1:1 gear ratio:

1. **Desired-velocity kS fade** (iter 16): kS scales with `|desiredVelocity| /
   peakVelocity`, fading to near-zero at cruise. Eliminates kS overdrive without
   relying on noisy actual-velocity measurements.

2. **PID-only output filter** (iter 13): EMA filter (α=0.3) applied only to the
   PID term, not feedforward. Smooths PID corrections without adding phase lag
   to the feedforward path.

3. **Asymmetric accel/decel** (iter 27): Separate acceleration (600°/s²) and
   deceleration (450°/s²) rates in the motion profile. The turret accelerates
   faster than it can brake.

4. **Distance-dependent accel scaling** (iter 29): Short moves (≤15°) use
   350°/s² accel, long moves (≥60°) use 600°/s², with linear interpolation
   between. Prevents CRServo velocity overshoot on short distances.

5. **peakVel normalization** (iter 19): kS fade uses the profile's actual peak
   velocity (which may be lower than maxVelocity for short triangular moves)
   ensuring consistent fade behavior across all move distances.

## Conclusion

The 1:1 gear ratio created significant tuning challenges because stiction
compensation dominated the drive signal at low speeds. However, these challenges
were overcome through a combination of:

- **Running faster** (280°/s vs 120°/s) — naturally increased kV·vel relative to
  kS, shifting from a kS-dominated to a kV-dominated regime
- **kS fade** — further reduced effective kS at cruise to 5.5% of total power
- **Specialized profile shaping** — asymmetric accel/decel and distance-dependent
  scaling addressed the remaining mechanical limitations

**Final iteration 29 results on 1:1 gearing:**
- Long moves (120–140°): 250–280°/s, 1.5–7° overshoot
- Short moves (20°): 76–110°/s, 0.1–5° overshoot

A higher gear ratio (e.g. 2:1) would still simplify the controller — likely P +
kV with no fade logic — but the current 1:1 ratio is producing production-quality
results with the software mitigations in place. **The performance achieved is
comparable to what was originally projected as requiring 3:1 gearing.**
