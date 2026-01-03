# FTC SDK 11.0.0: What getters use Lynx bulk caching?

This note summarizes, for **FTC SDK 11.0.0**, which high-level getters are served from the REV Hub **bulk read cache** (aka `LynxGetBulkInputData`) versus which ones still require their own transaction.

The “source of truth” here is the SDK implementation in the Gradle sources jars:

- `Hardware-11.0.0-sources.jar` (REV/Lynx implementations)
- `RobotCore-11.0.0-sources.jar` (public motor APIs that route into controllers)

## TL;DR

- With `LynxModule.BulkCachingMode` set to `AUTO` or `MANUAL`, only a small set of read commands are replaced by cached bulk values.
- On REV hubs in SDK 11.0.0, the bulk packet contains:
  - digital inputs
  - analog inputs
  - motor encoder positions
  - motor velocities
  - motor “at target” / busy
  - motor overcurrent flags
- Anything **I2C-based** (IMU, color/distance sensors, etc.) is **not** in the Lynx bulk packet.

## How bulk caching works (implementation-level)

When bulk caching is enabled (`AUTO`/`MANUAL`) and an eligible read method is called, the controller does roughly:

1. Ask the module to `recordBulkCachingCommandIntent(...)`
2. If the module has no cached bulk data yet, it issues a `LynxGetBulkInputDataCommand` to fetch it
3. It returns the requested field out of the cached `LynxModule.BulkData`

Important nuance:

- The **first eligible read** per hub per loop is typically the one that “pays” for the bulk read.
- In `MANUAL`, the cache only refreshes after you call `clearBulkCache()`.
- In `AUTO`, the cache refreshes when the *same eligible command* is issued twice (it compares payloads).

## What’s in `LynxModule.BulkData` (REV hubs)

In SDK 11.0.0, `LynxModule.BulkData` exposes these fields:

- `getDigitalChannelState(int digitalInputZ)`
- `getAnalogInputVoltage(int inputZ)` (and an overload with `VoltageUnit`)
- `getMotorCurrentPosition(int motorZ)`
- `getMotorVelocity(int motorZ)` (encoder counts per second)
- `isMotorBusy(int motorZ)` (implemented as “not at target”)
- `isMotorOverCurrent(int motorZ)`

If you’re looking for other “mystery getters”, this list is a good litmus test: if it’s not represented here, it’s not coming from a Lynx bulk packet.

## Motor getters: bulk-backed vs not

These are the most relevant ones for drivetrain loop time.

### Bulk-backed (when caching is `AUTO`/`MANUAL`)

**DcMotor / DcMotorEx** high-level calls that end up using `BulkData`:

- `DcMotor.getCurrentPosition()`
  - routes to `LynxDcMotorController.getMotorCurrentPosition()`
- `DcMotor.isBusy()`
  - routes to `LynxDcMotorController.isBusy()`
  - note: if bulk caching is OFF, the controller avoids NACK spam by returning `false` when not in `RUN_TO_POSITION`
- `DcMotorEx.getVelocity()` and `getVelocity(AngleUnit)`
  - routes to `LynxDcMotorController.getMotorVelocity()` → `internalGetMotorTicksPerSecond()`
  - uses a bulk input data read (with a per-motor tag)
- `DcMotorEx.isOverCurrent()`
  - routes to `LynxDcMotorController.isMotorOverCurrent()`
  - reads the overcurrent bit from the bulk packet

### Not bulk-backed (still separate transactions)

- `DcMotor.getPower()`
  - uses a dedicated “get motor constant power” command
- `DcMotor.getTargetPosition()`
  - uses a dedicated “get motor target position” command
- `DcMotorEx.getCurrent(CurrentUnit)`
  - uses a dedicated ADC command (`LynxGetADCCommand`)
- `DcMotorEx.getCurrentAlert(...)`
  - cached in software via `lastKnownCurrentAlert`, and/or fetched via a dedicated current alert level command

## Digital and analog: bulk-backed reads

These controller reads do use bulk caching when enabled:

- `AnalogInput.getVoltage()` (REV analog inputs)
- `DigitalChannel.getState()` **for input pins**

## Servos / PWM outputs

REV servo and PWM controllers do **not** appear to use `recordBulkCachingCommandIntent(...)` in SDK 11.0.0. In practice:

- there’s no “bulk read” data you can get for servo position; reads/writes go through their own paths
- logging servo/pwm state is generally not comparable to encoder/velocity reads in terms of bulk caching

## Practical guidance (PsiKit / logging)

- If you don’t need them, avoid per-loop reads of:
  - `getCurrentPosition()` / `isBusy()` / `getVelocity()`
  - even though they *can* be bulk-backed, they’re still not free (and the first read per hub pays the bulk cost)
- If you *do* need them, prefer patterns that minimize duplicates:
  - clear bulk once per loop (MANUAL)
  - then read all needed motor state once
- Expect “non-bulk” devices (I2C sensors, IMUs, some ADC-based reads like motor current) to remain comparatively expensive.

## Notes

- This document is specifically for **FTC SDK 11.0.0** and the REV/Lynx stack.
- Other motor controllers (non-Lynx) can behave differently.
