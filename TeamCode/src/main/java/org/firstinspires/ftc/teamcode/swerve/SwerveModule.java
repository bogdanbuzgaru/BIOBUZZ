package org.firstinspires.ftc.teamcode.swerve;

/**
 * One swerve module: a drive motor and a steering motor that rotates the whole
 * wheel assembly.
 *
 * Two things in here are not optional on a heavy robot:
 *
 *  1. Angle optimization. If the module is asked to turn more than 90 degrees,
 *     it instead turns to the opposite angle and runs the drive motor backwards.
 *     Same resulting wheel velocity, never more than a quarter turn of azimuth
 *     travel. Without this your modules spend their life doing 170 degree
 *     rotations while 50 kg of robot scrubs sideways.
 *
 *  2. Cosine compensation. While the module is still rotating into position, the
 *     drive output is scaled by cos(angleError). At 60 degrees off target you
 *     only get half power, at 90 degrees none. This stops the wheel from
 *     shoving the robot in the wrong direction during the transient, which is
 *     exactly when a heavy robot is most likely to skid or tip.
 */
public class SwerveModule {

    private final String name;

    private final SwerveHardware.MotorOutput driveMotor;
    private final SwerveHardware.MotorOutput steerMotor;
    private final SwerveHardware.AngleSensor steerAngle;
    /** May be null: then the drive runs purely on feedforward. */
    private final SwerveHardware.VelocitySensor driveVelocity;

    private final PIDController steerPid;
    private final PIDController drivePid;

    // drive feedforward, in units of motor output per (m/s) etc.
    private double kS = 0.0;   // output needed to just overcome friction
    private double kV = 0.0;   // output per metre per second
    private double kA = 0.0;   // output per metre per second squared

    /** Extra output to break steering gearbox stiction. Keep small. */
    private double steerKs = 0.0;

    private boolean cosineCompensation = true;

    // state
    private double lastCommandedSpeed = 0.0;
    private double lastTargetAngle = 0.0;
    private double lastDriveOutput = 0.0;
    private double lastSteerOutput = 0.0;

    public SwerveModule(String name,
                        SwerveHardware.MotorOutput driveMotor,
                        SwerveHardware.MotorOutput steerMotor,
                        SwerveHardware.AngleSensor steerAngle,
                        SwerveHardware.VelocitySensor driveVelocity,
                        PIDController steerPid,
                        PIDController drivePid) {
        this.name = name;
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerAngle = steerAngle;
        this.driveVelocity = driveVelocity;
        this.steerPid = steerPid;
        this.drivePid = drivePid;

        // Azimuth is circular. This is what makes "turn from 179 to -179" a two
        // degree move instead of a 358 degree one.
        this.steerPid.enableContinuousInput(-Math.PI, Math.PI);
        this.steerPid.setOutputRange(-1.0, 1.0);
    }

    public SwerveModule setDriveFeedforward(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    public SwerveModule setSteerStaticFeedforward(double steerKs) {
        this.steerKs = steerKs;
        return this;
    }

    public SwerveModule setCosineCompensation(boolean enabled) {
        this.cosineCompensation = enabled;
        return this;
    }

    /**
     * Command the module.
     *
     * @param speedMetersPerSec desired ground speed of this wheel
     * @param angleRadians      desired azimuth, field or robot frame already resolved
     * @param dt                seconds since the previous call
     */
    public void setDesiredState(double speedMetersPerSec, double angleRadians, double dt) {
        double current = steerAngle.getAngleRadians();

        // ---- 1. optimize: never travel more than 90 degrees of azimuth ----
        double target = SwerveHardware.normalize(angleRadians);
        double speed = speedMetersPerSec;
        double delta = SwerveHardware.normalize(target - current);

        if (Math.abs(delta) > Math.PI / 2.0) {
            target = SwerveHardware.normalize(target + Math.PI);
            speed = -speed;
            delta = SwerveHardware.normalize(target - current);
        }

        // If we are not being asked to move, hold the last azimuth rather than
        // snapping to zero. Modules twitching back to straight every time the
        // driver releases the stick wastes time and wears the gears.
        if (Math.abs(speed) < 1e-3) {
            target = lastTargetAngle;
            delta = SwerveHardware.normalize(target - current);
        } else {
            lastTargetAngle = target;
        }

        // ---- 2. steering loop ----
        double steerOutput = steerPid.calculate(target, current, dt);
        if (steerKs > 0.0 && Math.abs(steerOutput) > 1e-3) {
            steerOutput += Math.copySign(steerKs, steerOutput);
        }
        steerOutput = PIDController.clamp(steerOutput, -1.0, 1.0);
        steerMotor.set(steerOutput);
        lastSteerOutput = steerOutput;

        // ---- 3. cosine compensation ----
        if (cosineCompensation) {
            speed *= Math.cos(delta);
        }

        // ---- 4. drive loop: feedforward carries it, PID trims it ----
        double accel = (speed - lastCommandedSpeed) / Math.max(dt, 1e-4);
        double feedforward = kV * speed + kA * accel;
        if (Math.abs(speed) > 1e-3) {
            feedforward += Math.copySign(kS, speed);
        }

        double feedback = 0.0;
        if (driveVelocity != null) {
            feedback = drivePid.calculate(speed, driveVelocity.getVelocityMetersPerSec(), dt);
        }

        double driveOutput = PIDController.clamp(feedforward + feedback, -1.0, 1.0);
        driveMotor.set(driveOutput);

        lastDriveOutput = driveOutput;
        lastCommandedSpeed = speed;
    }

    /** Point the wheel somewhere without driving. Useful for an X-lock. */
    public void setAngleOnly(double angleRadians, double dt) {
        setAngleOnly(angleRadians, dt, true);
    }

    /**
     * Point the wheel somewhere without driving.
     *
     * @param allowFlip true to settle on the opposite angle when that is closer,
     *                  which is the same thing for driving. false to reach the
     *                  exact angle, e.g. for homing every wheel to forward.
     */
    public void setAngleOnly(double angleRadians, double dt, boolean allowFlip) {
        double current = steerAngle.getAngleRadians();
        double target = SwerveHardware.normalize(angleRadians);
        double delta = SwerveHardware.normalize(target - current);
        if (allowFlip && Math.abs(delta) > Math.PI / 2.0) {
            target = SwerveHardware.normalize(target + Math.PI);
        }
        lastTargetAngle = target;

        double out = steerPid.calculate(target, current, dt);
        if (steerKs > 0.0 && Math.abs(out) > 1e-3) {
            out += Math.copySign(steerKs, out);
        }
        steerMotor.set(PIDController.clamp(out, -1.0, 1.0));
        driveMotor.set(0.0);
        lastCommandedSpeed = 0.0;
    }

    public void stop() {
        driveMotor.set(0.0);
        steerMotor.set(0.0);
        lastCommandedSpeed = 0.0;
    }

    /** Clear controller state. Call on enable, or after the robot has been idle. */
    public void reset() {
        steerPid.reset();
        drivePid.reset();
        lastCommandedSpeed = 0.0;
        lastTargetAngle = steerAngle.getAngleRadians();
    }

    // ---- telemetry ----

    public String getName() {
        return name;
    }

    public double getAngleRadians() {
        return steerAngle.getAngleRadians();
    }

    public double getAngleDegrees() {
        return Math.toDegrees(steerAngle.getAngleRadians());
    }

    public double getVelocityMetersPerSec() {
        return driveVelocity == null ? 0.0 : driveVelocity.getVelocityMetersPerSec();
    }

    public double getSteerErrorDegrees() {
        return Math.toDegrees(steerPid.getError());
    }

    public double getDriveOutput() {
        return lastDriveOutput;
    }

    public double getSteerOutput() {
        return lastSteerOutput;
    }

    public PIDController getSteerPid() {
        return steerPid;
    }

    public PIDController getDrivePid() {
        return drivePid;
    }
}
