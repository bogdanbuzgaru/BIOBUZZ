package org.firstinspires.ftc.teamcode.swerve;

/**
 * The movement class. Takes a chassis velocity request and turns it into four
 * module states.
 *
 * Coordinate convention (standard robotics, matches WPILib):
 *   +x is forward, +y is to the robot's LEFT, +omega is counter-clockwise.
 *   Angles in radians, distances in metres, speeds in m/s.
 *
 * Module locations are measured from the centre of rotation, so a typical
 * 4-module layout is:
 *   front left  ( +halfLength, +halfWidth )
 *   front right ( +halfLength, -halfWidth )
 *   back left   ( -halfLength, +halfWidth )
 *   back right  ( -halfLength, -halfWidth )
 *
 * The mass-specific parts of this class are the acceleration limiter and the
 * heading hold. At 50 kg, a raw stick-to-motor mapping will out-request what the
 * wheels can transmit and the robot will skid, which breaks every assumption
 * your PID tuning rests on.
 */
public class SwerveDrive {

    private final SwerveModule[] modules;
    private final double[] moduleX;
    private final double[] moduleY;

    private final SwerveHardware.Gyro gyro;

    private double maxSpeedMetersPerSec;
    private double maxAngularSpeedRadPerSec;

    /** Limits the rate of change of the translation VECTOR, not each axis. */
    private double maxAccelMetersPerSecSq = Double.POSITIVE_INFINITY;
    private final SlewRateLimiter omegaLimiter;

    // heading hold
    private PIDController headingPid = null;
    private boolean headingHoldEnabled = false;
    private double headingSetpoint = 0.0;
    private boolean headingLatched = false;
    private double headingHoldDeadband = 0.03;

    private double deadband = 0.02;

    // state
    private double prevVx = 0.0;
    private double prevVy = 0.0;
    private long lastTimeNanos = 0L;

    public SwerveDrive(SwerveModule[] modules,
                       double[] moduleX,
                       double[] moduleY,
                       SwerveHardware.Gyro gyro,
                       double maxSpeedMetersPerSec,
                       double maxAngularSpeedRadPerSec) {
        if (modules.length != moduleX.length || modules.length != moduleY.length) {
            throw new IllegalArgumentException("modules and location arrays must be the same length");
        }
        this.modules = modules;
        this.moduleX = moduleX;
        this.moduleY = moduleY;
        this.gyro = gyro;
        this.maxSpeedMetersPerSec = maxSpeedMetersPerSec;
        this.maxAngularSpeedRadPerSec = maxAngularSpeedRadPerSec;
        this.omegaLimiter = new SlewRateLimiter(Double.POSITIVE_INFINITY);
    }

    // ------------------------------------------------------------------
    // configuration
    // ------------------------------------------------------------------

    /**
     * The single most useful knob for a heavy robot. Start conservative, around
     * 1.5 m/s^2, and raise it until the wheels start slipping on acceleration.
     */
    public SwerveDrive setMaxAcceleration(double metersPerSecSq) {
        this.maxAccelMetersPerSecSq = metersPerSecSq;
        return this;
    }

    public SwerveDrive setMaxAngularAcceleration(double radPerSecSq) {
        this.omegaLimiter.setRateLimit(radPerSecSq);
        return this;
    }

    /**
     * When the driver is not commanding rotation, a PID holds the last heading.
     * A 50 kg robot carries a lot of angular momentum, and module scrub is never
     * perfectly symmetric, so it will slowly yaw during a long translation unless
     * something actively corrects it.
     */
    public SwerveDrive enableHeadingHold(PIDController headingPid) {
        this.headingPid = headingPid;
        this.headingPid.enableContinuousInput(-Math.PI, Math.PI);
        this.headingPid.setOutputRange(-maxAngularSpeedRadPerSec, maxAngularSpeedRadPerSec);
        this.headingHoldEnabled = true;
        return this;
    }

    public SwerveDrive setHeadingHold(boolean enabled) {
        this.headingHoldEnabled = enabled && headingPid != null;
        if (!this.headingHoldEnabled) {
            headingLatched = false;
        }
        return this;
    }

    public SwerveDrive setDeadband(double deadband) {
        this.deadband = deadband;
        return this;
    }

    // ------------------------------------------------------------------
    // driving
    // ------------------------------------------------------------------

    /**
     * Convenience entry point for teleop. Inputs are joystick units in [-1, 1]
     * and get scaled to the configured max speeds. dt is measured internally.
     *
     * @param forward      +1 is away from the driver / robot forward
     * @param strafe       +1 is to the LEFT (negate your stick if it reports right-positive)
     * @param rotate       +1 is counter-clockwise
     * @param fieldCentric true to interpret forward/strafe in the field frame
     */
    public void driveNormalized(double forward, double strafe, double rotate, boolean fieldCentric) {
        forward = applyDeadband(forward);
        strafe = applyDeadband(strafe);
        rotate = applyDeadband(rotate);

        // Squaring keeps fine control at low stick travel, which matters a lot
        // when the thing you are nudging weighs 50 kg.
        forward = Math.copySign(forward * forward, forward);
        strafe = Math.copySign(strafe * strafe, strafe);
        rotate = Math.copySign(rotate * rotate, rotate);

        drive(forward * maxSpeedMetersPerSec,
                strafe * maxSpeedMetersPerSec,
                rotate * maxAngularSpeedRadPerSec,
                fieldCentric);
    }

    /** Real-units drive with internally measured dt. */
    public void drive(double vx, double vy, double omega, boolean fieldCentric) {
        drive(vx, vy, omega, fieldCentric, measureDt());
    }

    /**
     * Real-units drive.
     *
     * @param vx    forward speed, m/s
     * @param vy    leftward speed, m/s
     * @param omega counter-clockwise rotation, rad/s
     * @param dt    seconds since the previous call
     */
    public void drive(double vx, double vy, double omega, boolean fieldCentric, double dt) {

        // ---- 1. field centric transform, before any limiting ----
        if (fieldCentric) {
            if (gyro == null) {
                throw new IllegalStateException("field centric drive requires a gyro");
            }
            double heading = gyro.getYawRadians();
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double rotatedX = vx * cos - vy * sin;
            double rotatedY = vx * sin + vy * cos;
            vx = rotatedX;
            vy = rotatedY;
        }

        // ---- 2. heading hold ----
        if (headingHoldEnabled && gyro != null) {
            boolean driverIsRotating = Math.abs(omega) > headingHoldDeadband;
            boolean stillSpinning = Math.abs(gyro.getYawRateRadiansPerSec()) > 0.2;

            if (driverIsRotating || stillSpinning) {
                headingLatched = false;
                headingPid.reset();
            } else {
                if (!headingLatched) {
                    headingSetpoint = gyro.getYawRadians();
                    headingLatched = true;
                }
                omega = headingPid.calculate(headingSetpoint, gyro.getYawRadians(), dt);
            }
        }

        // ---- 3. acceleration limiting, on the vector ----
        // Limiting vx and vy independently would distort the direction of travel
        // during a diagonal move. Limiting the vector keeps the heading of the
        // requested velocity intact and only slows how fast its magnitude changes.
        if (maxAccelMetersPerSecSq != Double.POSITIVE_INFINITY && dt > 0) {
            double dvx = vx - prevVx;
            double dvy = vy - prevVy;
            double deltaMag = Math.hypot(dvx, dvy);
            double maxDelta = maxAccelMetersPerSecSq * dt;
            if (deltaMag > maxDelta && deltaMag > 1e-9) {
                double scale = maxDelta / deltaMag;
                vx = prevVx + dvx * scale;
                vy = prevVy + dvy * scale;
            }
        }
        prevVx = vx;
        prevVy = vy;

        omega = omegaLimiter.calculate(omega, dt);

        // ---- 4. inverse kinematics ----
        // Each module sits at (x, y) from the centre of rotation. Its velocity is
        // the chassis translation plus the tangential component of the rotation:
        //     v_i = v_chassis + omega x r_i
        // which in 2D is  vx_i = vx - omega*y_i ,  vy_i = vy + omega*x_i
        int n = modules.length;
        double[] speeds = new double[n];
        double[] angles = new double[n];

        for (int i = 0; i < n; i++) {
            double mvx = vx - omega * moduleY[i];
            double mvy = vy + omega * moduleX[i];
            speeds[i] = Math.hypot(mvx, mvy);
            // Keep the previous angle when the module is asked for zero speed,
            // otherwise atan2(0,0) snaps it to 0 radians.
            angles[i] = (speeds[i] > 1e-4)
                    ? Math.atan2(mvy, mvx)
                    : modules[i].getAngleRadians();
        }

        // ---- 5. desaturate ----
        // Rotation plus translation can ask a module for more than the drivetrain
        // can deliver. Scaling every module by the same factor preserves the shape
        // of the motion instead of clipping one wheel and skewing the path.
        double maxRequested = 0.0;
        for (double s : speeds) {
            maxRequested = Math.max(maxRequested, s);
        }
        if (maxRequested > maxSpeedMetersPerSec) {
            double scale = maxSpeedMetersPerSec / maxRequested;
            for (int i = 0; i < n; i++) {
                speeds[i] *= scale;
            }
        }

        // ---- 6. push to modules ----
        for (int i = 0; i < n; i++) {
            modules[i].setDesiredState(speeds[i], angles[i], dt);
        }
    }

    /** Robot-relative shorthand. */
    public void driveRobotCentric(double vx, double vy, double omega) {
        drive(vx, vy, omega, false);
    }

    /**
     * Point all modules at the centre of the robot so it resists being pushed.
     * Worth having: 50 kg is enough that you do not want to rely on brake mode
     * alone to hold position.
     */
    public void lockWheels() {
        double dt = measureDt();
        for (int i = 0; i < modules.length; i++) {
            double angle = Math.atan2(moduleY[i], moduleX[i]) + Math.PI / 2.0;
            modules[i].setAngleOnly(angle, dt);
        }
        prevVx = 0.0;
        prevVy = 0.0;
        omegaLimiter.reset(0.0);
    }

    public void stop() {
        for (SwerveModule m : modules) {
            m.stop();
        }
        prevVx = 0.0;
        prevVy = 0.0;
        omegaLimiter.reset(0.0);
    }

    /** Call once on enable, and after any pause in the control loop. */
    public void reset() {
        for (SwerveModule m : modules) {
            m.reset();
        }
        if (headingPid != null) {
            headingPid.reset();
        }
        headingLatched = false;
        prevVx = 0.0;
        prevVy = 0.0;
        omegaLimiter.reset(0.0);
        lastTimeNanos = 0L;
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public double getMaxSpeedMetersPerSec() {
        return maxSpeedMetersPerSec;
    }

    // ------------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------------

    private double applyDeadband(double value) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        // rescale so the output is still continuous at the deadband edge
        return (value - Math.copySign(deadband, value)) / (1.0 - deadband);
    }

    private double measureDt() {
        long now = System.nanoTime();
        if (lastTimeNanos == 0L) {
            lastTimeNanos = now;
            return 0.02;
        }
        double dt = (now - lastTimeNanos) / 1e9;
        lastTimeNanos = now;
        // A long stall (loading, a blocking call) should not be fed to the D term.
        return PIDController.clamp(dt, 1e-4, 0.1);
    }
}
