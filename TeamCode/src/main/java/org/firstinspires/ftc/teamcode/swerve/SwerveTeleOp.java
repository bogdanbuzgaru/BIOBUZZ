package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * Runnable TeleOp. Unlike ExampleUsage, every SDK call here is live.
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

    private static final String[] NAMES = {"fl", "fr", "bl", "br"};

    // ---------------- BRING-UP VALUES: find these on the robot ----------------

    /** True if a positive drive power moves that corner BACKWARD. */
    private static final boolean[] DRIVE_INVERTED = {false, false, false, false};

    /** True if a positive steer power rotates the module CLOCKWISE. */
    private static final boolean[] STEER_INVERTED = {false, false, false, false};

    /** True if rotating a module counter-clockwise makes its tick count FALL. */
    private static final boolean[] AZIMUTH_INVERTED = {false, false, false, false};

    /** Match these to how the Control Hub is physically mounted. */
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // --------------------------------------------------------------------------

    private final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private final DcMotorEx[] steerMotors = new DcMotorEx[4];
    private final ThroughBoreSteeringEncoder[] azimuth = new ThroughBoreSteeringEncoder[4];
    private List<LynxModule> hubs;
    private IMU imu;
    private SwerveDrive drive;
    private ThroughBoreSteeringEncoder.HomingRoutine homing;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---- bulk caching ----
        // Without this, every getCurrentPosition() and getVelocity() is a
        // separate round trip to the hub. With 8 motors across two hubs that is
        // 16+ transactions per loop and loop times climb past 50 ms, which makes
        // the derivative term useless and the whole drivetrain feel mushy.
        // MANUAL mode reads each hub once per loop instead.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        SwerveModule[] modules = new SwerveModule[4];

        for (int i = 0; i < 4; i++) {
            driveMotors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i] + "Drive");
            steerMotors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i] + "Steer");

            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            steerMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            steerMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // BRAKE on both. 50 kg coasting is 50 kg you are not steering.
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            steerMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (DRIVE_INVERTED[i]) {
                driveMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (STEER_INVERTED[i]) {
                steerMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }

            final int k = i;

            SwerveHardware.MotorOutput driveOut = power -> driveMotors[k].setPower(power);
            SwerveHardware.MotorOutput steerOut = power -> steerMotors[k].setPower(power);

            SwerveHardware.VelocitySensor driveVel = new SwerveHardware.EncoderVelocitySensor(
                    () -> driveMotors[k].getVelocity(),       // ticks per second
                    SwerveConstants.DRIVE_TICKS_PER_REV,
                    SwerveConstants.DRIVE_GEAR_RATIO,
                    SwerveConstants.WHEEL_CIRCUMFERENCE_M);

            // The Through Bore lives on the steering motor's encoder port, so it
            // is read through that motor object. The steering motor's own
            // internal encoder is simply unused.
            azimuth[i] = new ThroughBoreSteeringEncoder(
                    () -> steerMotors[k].getCurrentPosition(),
                    SwerveConstants.AZIMUTH_GEAR_RATIO,       // 1.0, encoder is on the axis
                    AZIMUTH_INVERTED[i]);

            PIDController steerPid = new PIDController(
                    SwerveConstants.STEER_KP, SwerveConstants.STEER_KI, SwerveConstants.STEER_KD)
                    .setTolerance(SwerveConstants.STEER_TOLERANCE_RAD)
                    .setIZone(Math.toRadians(15))
                    .setMaxIntegralContribution(0.15)
                    .setDerivativeFilter(0.5);

            PIDController drivePid = new PIDController(
                    SwerveConstants.DRIVE_KP, SwerveConstants.DRIVE_KI, SwerveConstants.DRIVE_KD)
                    .setOutputRange(-0.4, 0.4)
                    .setIZone(0.3)
                    .setMaxIntegralContribution(0.1);

            modules[i] = new SwerveModule(NAMES[i], driveOut, steerOut,
                    azimuth[i], driveVel, steerPid, drivePid)
                    .setDriveFeedforward(SwerveConstants.DRIVE_KS,
                            SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA)
                    .setSteerStaticFeedforward(SwerveConstants.STEER_KS)
                    .setCosineCompensation(true);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(LOGO_DIR, USB_DIR)));
        imu.resetYaw();

        SwerveHardware.Gyro gyro = new SwerveHardware.Gyro() {
            @Override
            public double getYawRadians() {
                return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            @Override
            public double getYawRateRadiansPerSec() {
                return imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
            }
        };

        drive = new SwerveDrive(modules, SwerveConstants.MODULE_X, SwerveConstants.MODULE_Y,
                gyro, SwerveConstants.MAX_SPEED_MPS, SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC)
                .setMaxAcceleration(SwerveConstants.MAX_ACCEL_MPS2)
                .setMaxAngularAcceleration(SwerveConstants.MAX_ANGULAR_ACCEL_RAD_PER_SEC2)
                .enableHeadingHold(new PIDController(SwerveConstants.HEADING_KP,
                        SwerveConstants.HEADING_KI, SwerveConstants.HEADING_KD));

        homing = new ThroughBoreSteeringEncoder.HomingRoutine(azimuth, modules);
        homing.relax();

        // ---------------- INIT: align the wheels by hand ----------------
        boolean homed = false;
        while (opModeInInit()) {
            clearCaches();
            if (gamepad1.a) {
                // Offsets are all zero unless you set them in SwerveConstants,
                // in which case you align to your reference rather than forward.
                homing.homeAll(SwerveConstants.azimuthOffsetsRadians());
                imu.resetYaw();
                homed = true;
            }
            telemetry.addLine(homed
                    ? "HOMED. Press PLAY."
                    : "Align all four wheels, then press A.");
            for (int i = 0; i < 4; i++) {
                telemetry.addData(NAMES[i], "raw %8.0f   angle %7.2f deg   offset %5.1f",
                        azimuth[i].getRawTicks(), azimuth[i].getAngleDegrees(),
                        SwerveConstants.AZIMUTH_OFFSET_DEG[i]);
            }
            telemetry.addLine("Run Swerve Calibration first if any flag is unknown.");
            telemetry.addData("top speed", "%.2f m/s", SwerveConstants.MAX_SPEED_MPS);
            telemetry.addData("sustainable accel", "%.2f m/s2",
                    SwerveConstants.sustainableAccelMps2());
            telemetry.update();
        }

        if (isStopRequested()) return;

        if (!homed) {
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
            clearCaches();

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
                imu.resetYaw();
            }

            for (SwerveModule m : drive.getModules()) {
                telemetry.addData(m.getName(), "ang %6.1f  err %5.1f  v %5.2f  out %5.2f",
                        m.getAngleDegrees(), m.getSteerErrorDegrees(),
                        m.getVelocityMetersPerSec(), m.getDriveOutput());
            }
            telemetry.update();
        }

        drive.stop();
    }

    /** Must run once at the top of every loop, or the bulk cache goes stale. */
    private void clearCaches() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
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
     *   azimuth[k].seedFrom(abs);
     *
     * That replaces the hand-alignment step while keeping the quadrature
     * output's 8192-count resolution. Library notes:
     *   OctoQuad MK1 - driver is built into FTC SDK v9.2 and later, nothing to add.
     *   OctoQuad MK2 - the built-in driver is NOT compatible; add the driver
     *                  source from DigitalChickenLabs' repository to TeamCode.
     */
}
