package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Drive-only swerve TeleOp, for bring-up and tuning. The competition TeleOp
 * (opmodes/teleop/TeleOp) drives the same way and also runs the mechanisms.
 *
 * Place this package at:
 *   TeamCode/src/main/java/org/firstinspires/ftc/teamcode/swerve/
 *
 * No external libraries are required for this version. It reads the REV Through
 * Bore encoders as QUADRATURE through the steering motors' encoder ports, so
 * everything it needs is already in the FTC SDK. If you later add an OctoQuad to
 * read the absolute output, see the note at the bottom of this file.
 *
 * CONFIGURATION FILE NAMES (create these on the Driver Station):
 *   Control Hub   motor 0  flDrive      Expansion Hub  motor 0  flSteer
 *                 motor 1  frDrive                     motor 1  frSteer
 *                 motor 2  blDrive                     motor 2  blSteer
 *                 motor 3  brDrive                     motor 3  brSteer
 *   Control Hub   I2C 0    imu   (built in, type "REV Internal IMU")
 *
 * Each module's Through Bore plugs into the ENCODER port beside its steering
 * motor, via the REV-31-1815 6-pin to 4-pin cable, with the encoder's mode
 * switch set to 'A'.
 */
@TeleOp(name = "Swerve TeleOp", group = "drive")
public class SwerveTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SwerveRobot robot = new SwerveRobot(hardwareMap);
        SwerveDrive drive = robot.getDrive();

        // ---------------- INIT: align the wheels by hand ----------------
        while (opModeInInit()) {
            robot.clearBulkCache();
            if (gamepad1.a) {
                robot.home();
            }
            robot.addHomingTelemetry(telemetry);
            telemetry.addLine("Run Swerve Calibration first if any flag is unknown.");
            telemetry.addData("top speed", "%.2f m/s", SwerveConstants.MAX_SPEED_MPS);
            telemetry.addData("sustainable accel", "%.2f m/s2",
                    SwerveConstants.sustainableAccelMps2());
            telemetry.update();
        }

        if (isStopRequested()) return;

        if (!robot.isHomed()) {
            // Refusing to move is the correct behaviour. An unhomed 50 kg swerve
            // drive will pick a direction of its own.
            while (opModeIsActive()) {
                drive.stop();
                telemetry.addLine("NOT HOMED - restart the OpMode and press A in init.");
                telemetry.update();
            }
            return;
        }

        drive.reset();

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive()) {
            robot.clearBulkCache();

            if (gamepad1.left_bumper) {
                drive.lockWheels();
            } else {
                // Sticks report y-down and x-right; negate both for +x forward, +y left.
                drive.driveNormalized(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        !gamepad1.right_bumper);   // hold RB for robot-centric
            }

            if (gamepad1.back) {
                robot.resetYaw();
            }

            robot.addModuleTelemetry(telemetry);
            telemetry.update();
        }

        drive.stop();
    }

    /*
     * IF YOU ADD AN OCTOQUAD LATER
     *
     * Wire each Through Bore's 3-pin PWM lead (REV-11-1817) to an OctoQuad
     * channel and the OctoQuad to any I2C port. Then, once during init:
     *
     *   PulseWidthAbsoluteEncoder abs = PulseWidthAbsoluteEncoder.throughBoreV2(
     *           () -> octoquad.readSinglePosition(k),   // microseconds
     *           ABS_OFFSET_RAD[k], false);
     *   azimuth[k].seedFrom(abs);      // in SwerveRobot's constructor
     *
     * That replaces the hand-alignment step while keeping the quadrature
     * output's 8192-count resolution. Library notes:
     *   OctoQuad MK1 - driver is built into FTC SDK v9.2 and later, nothing to add.
     *   OctoQuad MK2 - the built-in driver is NOT compatible; add the driver
     *                  source from DigitalChickenLabs' repository to TeamCode.
     */
}
