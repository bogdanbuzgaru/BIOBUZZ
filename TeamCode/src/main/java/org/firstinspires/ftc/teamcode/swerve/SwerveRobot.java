package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

/**
 * All the swerve hardware in one place: the eight motors, the four Through Bore
 * azimuth encoders, the IMU and the SwerveDrive built on top of them.
 *
 * Every OpMode that drives the robot builds one of these, so the hardware setup
 * lives in exactly one file. Hardware names are in the README.
 *
 * Bulk caching is MANUAL: call clearBulkCache() once at the top of every loop,
 * or the encoder readings go stale. PedroSwerveDrivetrain does this for you.
 */
public class SwerveRobot {

    private final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private final DcMotorEx[] steerMotors = new DcMotorEx[4];
    private final ThroughBoreSteeringEncoder[] azimuth = new ThroughBoreSteeringEncoder[4];
    private final SwerveModule[] modules = new SwerveModule[4];
    private final List<LynxModule> hubs;
    private final IMU imu;
    private final SwerveDrive drive;
    private final ThroughBoreSteeringEncoder.HomingRoutine homing;

    public SwerveRobot(HardwareMap hardwareMap) {

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

        String[] names = SwerveConstants.MODULE_NAMES;
        for (int i = 0; i < 4; i++) {
            driveMotors[i] = hardwareMap.get(DcMotorEx.class, names[i] + "Drive");
            steerMotors[i] = hardwareMap.get(DcMotorEx.class, names[i] + "Steer");

            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            steerMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            steerMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // BRAKE on both. 50 kg coasting is 50 kg you are not steering.
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            steerMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (SwerveConstants.DRIVE_INVERTED[i]) {
                driveMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (SwerveConstants.STEER_INVERTED[i]) {
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
                    SwerveConstants.AZIMUTH_INVERTED[i]);

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

            modules[i] = new SwerveModule(names[i], driveOut, steerOut,
                    azimuth[i], driveVel, steerPid, drivePid)
                    .setDriveFeedforward(SwerveConstants.DRIVE_KS,
                            SwerveConstants.DRIVE_KV, SwerveConstants.DRIVE_KA)
                    .setSteerStaticFeedforward(SwerveConstants.STEER_KS)
                    .setCosineCompensation(true);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(SwerveConstants.LOGO_DIR, SwerveConstants.USB_DIR)));
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
    }

    /** Must run once at the top of every loop, or the bulk cache goes stale. */
    public void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    /**
     * Declare the wheels aligned. Call with every module pointed straight
     * forward, or at its reference if AZIMUTH_OFFSET_DEG is set. Also zeroes the
     * IMU, so field-centric "forward" is the way the robot faces right now.
     */
    public void home() {
        // Offsets are all zero unless you set them in SwerveConstants, in which
        // case you align to your reference rather than forward.
        homing.homeAll(SwerveConstants.azimuthOffsetsRadians());
        imu.resetYaw();
        drive.reset();
    }

    public boolean isHomed() {
        return homing.allHomed();
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public SwerveDrive getDrive() {
        return drive;
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    /** Raw encoder readout for the init screen, while the wheels are being aligned. */
    public void addHomingTelemetry(Telemetry telemetry) {
        telemetry.addLine(isHomed()
                ? "HOMED. Press PLAY."
                : "Align all four wheels, then press A.");
        for (int i = 0; i < 4; i++) {
            telemetry.addData(SwerveConstants.MODULE_NAMES[i],
                    "raw %8.0f   angle %7.2f deg   offset %5.1f",
                    azimuth[i].getRawTicks(), azimuth[i].getAngleDegrees(),
                    SwerveConstants.AZIMUTH_OFFSET_DEG[i]);
        }
    }

    public void addModuleTelemetry(Telemetry telemetry) {
        for (SwerveModule m : modules) {
            telemetry.addData(m.getName(), "ang %6.1f  err %5.1f  v %5.2f  out %5.2f",
                    m.getAngleDegrees(), m.getSteerErrorDegrees(),
                    m.getVelocityMetersPerSec(), m.getDriveOutput());
        }
    }
}
