# FTC Swerve Drive

Four modules, each with a goBILDA 1150 RPM drive motor (4.8:1 external) and a
312 RPM steering motor (2.955:1), REV Through Bore encoders read as quadrature
on the azimuth axis. Tuned around a 50 kg robot on 96 mm wheels.

## Install

Extract so the files land at:

    TeamCode/src/main/java/org/firstinspires/ftc/teamcode/swerve/

No external libraries. Everything used is in the stock FTC SDK.

## Hardware configuration names

    Control Hub    motor 0..3   flDrive frDrive blDrive brDrive
    Expansion Hub  motor 0..3   flSteer frSteer blSteer brSteer
    Control Hub    I2C 0        imu   (REV Internal IMU)

Each module's Through Bore plugs into the ENCODER port beside its steering
motor (REV-31-1815 6-pin to 4-pin cable, encoder mode switch on 'A'). The
absolute 3-pin lead is left unplugged — the hub cannot decode it.

## Order of operations

1. Run `Swerve Calibration` on blocks. Settle the three boolean arrays at the
   top of `SwerveTeleOp`: DRIVE_INVERTED, STEER_INVERTED, AZIMUTH_INVERTED.
   Check AZIMUTH_INVERTED on blocks — a wrong sign makes a module run away.
2. Set LOGO_DIR and USB_DIR to match how the Control Hub is mounted.
3. Run `Swerve TeleOp`. Align wheels forward in init, press A, then PLAY.

## Tuning order

1. Azimuth kP on blocks, then add kD. Leave kI at zero; use STEER_KS if it
   stops short.
2. Repeat at full weight on the floor — it will want more kP.
3. Drive kV: command fixed power, measure steady speed, kV = power / speed.
   kS: raise power from zero until the wheels just turn.
4. Then drive kP, output kept clamped.
5. Last, raise MAX_ACCEL_MPS2 until the wheels slip, then back off 20%.

## Known open items

- Loop time must stay under ~15 ms or the D terms are noise. Bulk caching is
  set to MANUAL and cleared once per loop; do not remove that.
- MAX_ANGULAR_SPEED derives to ~230 deg/s, which may feel twitchy at 50 kg.
- Persistent azimuth offsets are not possible on the quadrature path. An
  OctoQuad reading the absolute pulse output would remove the homing step —
  see PulseWidthAbsoluteEncoder and the note at the bottom of SwerveTeleOp.
