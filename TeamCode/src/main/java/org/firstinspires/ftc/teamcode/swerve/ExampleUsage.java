package org.firstinspires.ftc.teamcode.swerve;

/**
 * How the pieces fit together. Written against the SwerveHardware interfaces,
 * with the FTC SDK calls shown in comments so you can see where they go.
 *
 * In an FTC OpMode this would be split between init() and the loop.
 */
public class ExampleUsage {

    private SwerveDrive drive;

    public void init(/* HardwareMap hardwareMap */) {

        String[] names = {"fl", "fr", "bl", "br"};
        SwerveModule[] modules = new SwerveModule[4];

        for (int i = 0; i < 4; i++) {
            // --- FTC SDK ---
            // DcMotorEx driveM = hardwareMap.get(DcMotorEx.class, names[i] + "Drive");
            // DcMotorEx steerM = hardwareMap.get(DcMotorEx.class, names[i] + "Steer");
            // driveM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // steerM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            // driveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // steerM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            //
            // SwerveHardware.MotorOutput driveOut = driveM::setPower;
            // SwerveHardware.MotorOutput steerOut = steerM::setPower;
            //
            // SwerveHardware.VelocitySensor driveVel = new SwerveHardware.EncoderVelocitySensor(
            //         driveM::getVelocity,
            //         SwerveConstants.DRIVE_TICKS_PER_REV,
            //         SwerveConstants.DRIVE_GEAR_RATIO,
            //         SwerveConstants.WHEEL_CIRCUMFERENCE_M);
            //
            // Absolute steering sensor, strongly preferred:
            // AnalogInput abs = hardwareMap.get(AnalogInput.class, names[i] + "Abs");
            // SwerveHardware.AngleSensor steerAngle = new SwerveHardware.AnalogAbsoluteAngleSensor(
            //         abs::getVoltage, abs::getMaxVoltage, ABS_OFFSETS[i], false);
            //
            // Relative fallback, requires aligning the wheels before every run:
            // SwerveHardware.AngleSensor steerAngle = new SwerveHardware.EncoderAngleSensor(
            //         () -> steerM.getCurrentPosition(),
            //         SwerveConstants.STEER_TICKS_PER_REV,
            //         SwerveConstants.AZIMUTH_GEAR_RATIO);

            SwerveHardware.MotorOutput driveOut = power -> { /* driveM.setPower(power) */ };
            SwerveHardware.MotorOutput steerOut = power -> { /* steerM.setPower(power) */ };
            SwerveHardware.AngleSensor steerAngle = () -> 0.0;
            SwerveHardware.VelocitySensor driveVel = () -> 0.0;

            PIDController steerPid = new PIDController(
                    SwerveConstants.STEER_KP,
                    SwerveConstants.STEER_KI,
                    SwerveConstants.STEER_KD)
                    .setTolerance(SwerveConstants.STEER_TOLERANCE_RAD)
                    .setIZone(Math.toRadians(15))
                    .setMaxIntegralContribution(0.15)
                    .setDerivativeFilter(0.3);

            PIDController drivePid = new PIDController(
                    SwerveConstants.DRIVE_KP,
                    SwerveConstants.DRIVE_KI,
                    SwerveConstants.DRIVE_KD)
                    .setOutputRange(-0.4, 0.4)   // feedback only trims the feedforward
                    .setIZone(0.3)
                    .setMaxIntegralContribution(0.1);

            modules[i] = new SwerveModule(names[i], driveOut, steerOut, steerAngle, driveVel,
                    steerPid, drivePid)
                    .setDriveFeedforward(
                            SwerveConstants.DRIVE_KS,
                            SwerveConstants.DRIVE_KV,
                            SwerveConstants.DRIVE_KA)
                    .setSteerStaticFeedforward(SwerveConstants.STEER_KS)
                    .setCosineCompensation(true);
        }

        // IMU imu = hardwareMap.get(IMU.class, "imu");
        SwerveHardware.Gyro gyro = new SwerveHardware.Gyro() {
            @Override
            public double getYawRadians() {
                return 0.0;   // imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)
            }

            @Override
            public double getYawRateRadiansPerSec() {
                return 0.0;   // imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate
            }
        };

        drive = new SwerveDrive(
                modules,
                SwerveConstants.MODULE_X,
                SwerveConstants.MODULE_Y,
                gyro,
                SwerveConstants.MAX_SPEED_MPS,
                SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC)
                .setMaxAcceleration(SwerveConstants.MAX_ACCEL_MPS2)
                .setMaxAngularAcceleration(SwerveConstants.MAX_ANGULAR_ACCEL_RAD_PER_SEC2)
                .enableHeadingHold(new PIDController(
                        SwerveConstants.HEADING_KP,
                        SwerveConstants.HEADING_KI,
                        SwerveConstants.HEADING_KD));

        drive.reset();
    }

    /** Call this every loop iteration, as fast and as evenly as you can. */
    public void loop(double stickLeftY, double stickLeftX, double stickRightX, boolean lockButton) {
        if (lockButton) {
            drive.lockWheels();
            return;
        }

        // Gamepad sticks report y-down and x-right, so both are negated to match
        // the +x forward / +y left convention.
        drive.driveNormalized(
                -stickLeftY,    // forward
                -stickLeftX,    // left
                -stickRightX,   // counter-clockwise
                true);          // field centric

        // Telemetry worth having while tuning:
        // for (SwerveModule m : drive.getModules()) {
        //     telemetry.addData(m.getName(),
        //             "ang %.1f  err %.1f  v %.2f  out %.2f",
        //             m.getAngleDegrees(), m.getSteerErrorDegrees(),
        //             m.getVelocityMetersPerSec(), m.getDriveOutput());
        // }
    }

    public void stop() {
        drive.stop();
    }
}