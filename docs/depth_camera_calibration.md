# Astra depth calibration

Whether a metre reported by the depth camera is a metre, how noisy it is, and
where it stops working. [Measured sensor capabilities](sensor_capabilities.md)
covers what the camera delivers and how fast; this covers whether to believe
it.

Reproduce with `tools/depth_wall_calibration.py`, one run per position:

```bash
python3 tools/depth_wall_calibration.py --tape 1.00 --tape-case 0.075
```

## Measurement record

| | |
|---|---|
| Robot | `x3-c`, Astra `ACRC64300ET`, pinned Orbbec driver `f7e71d9` |
| Date | 2026-09-18 |
| Commit | `01e1a60` (`main`) |
| Target | Flat interior wall, a table part-way between robot and wall |
| Reference | Tape from the chassis front; robot pushed by hand between positions |
| Tape | Truper FH-5M, 75 mm case (see [tape case](#the-tape-case)) |

## Why the wall is fitted rather than sampled

The obvious method -- read the centre pixel, compare against a tape -- does not
work, and fails in a way that looks convincing.

A robot pushed by hand is never square to the wall. Depth read off a slanted
surface is the distance to that slant. On the first position here, a 16 degree
yaw turned a true `-1.2%` error into an apparent **`+5.7%`**, with the sign
flipped. Nothing in that reading looks wrong.

Fitting the plane recovers the perpendicular distance, the yaw, the camera's
pitch, and the residual scatter, which is the depth noise with the geometry
taken out. The fit is RANSAC rather than least squares because the room had a
table between the robot and the wall: a second plane with fewer points, which
least squares averages into a surface that exists nowhere. RANSAC identified
the table as `31.5%` off-plane points at `2.86 m` and excluded it without
being told it was there.

The fit is repeatable to well under a millimetre. Three seeds at one position
returned `1.6348 / 1.6348 / 1.6348 m`.

### The tape case

A retracted tape measure's case sits between the surface and the zero mark, so
every reading is short by the case length. It is a constant offset, which is
exactly what a depth bias looks like, and it dominated the first analysis here.

Fitting `reported = a * (tape + chassis offset) + b` over the four
low-yaw positions gave `a = 0.98674`, `b = 0.07594`, with residuals under
`0.9 mm`. Solving for the case length that leaves no constant term gives
**`77.0 mm`**. The tape in use is a Truper FH-5M whose published housing is
`7.5 x 7.5 x 3.5 cm` -- **`75 mm`**, agreeing to `2 mm` with a figure derived
purely from depth data.

Because the case profile is square, its orientation does not matter. `75 mm`
is used below.

## Scale

Positions with low yaw, where it makes no difference whether the tape ran
perpendicular to the wall or along the robot's axis:

| Tape | Truth | Reported | Error |
|---|---|---|---|
| 0.50 m | 0.6344 m | 0.6284 m | **-0.94%** |
| 0.70 m | 0.8344 m | 0.8252 m | **-1.10%** |
| 1.00 m | 1.1344 m | 1.1204 m | **-1.23%** |
| 1.52 m | 1.6544 m | 1.6348 m | **-1.18%** |

**The camera under-reports depth by about 1.1%.** The linear fit's slope puts
it at `1.33%`; the spread across positions is `0.3%`. Record it as
**`-1.1%` to `-1.3%`**, and correct with `true ~= reported * 1.012`.

Two further positions at `2.52 m` and `3.52 m` had `7.8` and `8.4` degrees of
yaw, where the tape's direction matters: they give `-1.9%` and `-1.6%` read as
perpendicular, or `-1.0%` and `-0.5%` read as along-axis. Consistent with the
above, but not independent evidence. Keep the robot within a few degrees of
square for any position meant to constrain scale.

A `1.1%` scale error is small but systematic, not noise: over a 10 m run it
accumulates 11 cm.

## Noise

Residual RMS about the fitted plane, which excludes tilt and surface shape:

| Distance | Noise |
|---|---|
| 0.45 m | 0.13 cm |
| 0.63 m | 0.19 cm |
| 0.83 m | 0.20 cm |
| 1.12 m | 0.25 cm |
| 1.63 m | 0.57 cm |
| 2.60 m | 1.84 cm |
| 3.60 m | 3.93 cm |

Above roughly `1.1 m` this is a clean power law:

```
sigma ~= 0.0019 * d^2.36    metres
```

Four points across a `3.2x` range span fit that to within `7%`. Below `1.1 m`
the law over-predicts how good the sensor gets: measured noise flattens at a
floor of about `2 mm`. So:

```
sigma ~= max(0.002, 0.0019 * d^2.36)    metres
```

The exponent matters. A textbook `d^2` model understates noise at `3.5 m` by
roughly `40%`, and a constant-noise model is wrong by `16x` across this span.

## Range limits and dropout

**Usable range is about `0.6 m` to at least `3.6 m`.** Below `0.6 m` the
sensor degrades in two ways at once:

| Reported | Valid pixels | Scale error |
|---|---|---|
| 0.63 m | 61.1% | `-0.94%`, on model |
| 0.45 m | 35.4% | `-8.45%`, off model |

No tape-case value reconciles the `0.45 m` point with the others; it would
need `38.9 mm` where every other position needs `75 mm`. The camera is simply
outside its working range, which matches the Astra family's published `0.6 m`
minimum.

The close-range failure is **not** a range cutoff. Every surviving pixel at
`0.45 m` read between `0.447` and `0.473 m`, so a distance threshold would
have removed them all. Instead the dead zone is a diagonal wedge sweeping in
from the left:

```
valid pixels per cell, 0.45 m from the wall
        c0    c1    c2    c3    c4    c5    c6    c7
row 0 :  0%    4%   74%   82%   85%   85%   85%   41%
row 1 :  0%    0%   12%   77%  100%  100%  100%   47%
row 2 :  0%    0%    0%    3%   79%  100%  100%   48%
row 3 :  0%    0%    0%    0%   41%  100%  100%   48%
row 4 :  0%    0%    0%    0%   13%   73%   71%   31%
row 5 :  0%    0%    0%    0%    0%    0%    0%    0%
```

This is the structured-light baseline: the IR projector sits beside the IR
camera, and at close range their cones stop overlapping on one side, so that
part of the frame receives no pattern to decode. It is geometry, and it is
systematic -- the same wedge appears in the same place every time.

Two other standing dropouts, both visible above:

- **The floor never returns.** The bottom third of the frame is empty at every
  distance tested.
- **Valid coverage peaks around `0.6-0.8 m`** and falls off with range: `61%`
  at `0.63 m`, `57%` at `1.12 m`, `48%` at `3.60 m`.

## What a simulator has to copy

A simulated depth camera returns dense depth across its whole frustum, at the
exact distance, with uniform or no noise. Every one of those is wrong here:

- apply a **`-1.1%` scale**;
- apply **`sigma = max(0.002, 0.0019 * d^2.36)`**, not constant noise;
- return **nothing below `0.6 m`**, and expect a **wedge-shaped dead zone on
  one side** as a surface gets close;
- return depth for only **half the frame** in a typical scene;
- never return the floor.

The close-range wedge is the one most likely to matter. It is worst exactly
when a robot is approaching an obstacle, which is when a consumer written
against the simulator will most expect depth to be there.

## Limitations

- **One camera, one robot, one room.** Nothing here establishes unit-to-unit
  variation.
- **Ground truth is a hand tape**, roughly `+/- 1 cm`, and the robot was
  pushed by hand between positions. The `0.3%` spread in the scale figure is
  consistent with that alone.
- **The `75 mm` case length is from the manufacturer's published housing
  dimensions**, not a caliper. It agrees with the value derived from the depth
  data to `2 mm`, which is why it is trusted, but both could share a common
  error.
- **One surface, one material.** A white interior wall is close to the best
  case for structured light. Dark, glossy, or transparent surfaces will drop
  out far more, and none were tested.
- **No lighting variation.** All measurements were taken under the same
  indoor lighting; IR-rich sunlight is known to degrade this sensor class and
  was not tested.
- **Nothing here was measured in motion.** Rolling shutter, motion blur, and
  vibration are all unmeasured.
