package org.firstinspires.ftc.teamcode.swerve;

/**
 * Thin hardware abstraction so the control code does not depend on any
 * particular SDK. Implement these with three or four lines each.
 *
 * FTC SDK example:
 *
 *   DcMotorEx drive = hardwareMap.get(DcMotorEx.class, "flDrive");
 *   drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);   // we close our own loop
 *   drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
 *
 *   SwerveHardware.MotorOutput driveOut = drive::setPower;
 *
 *   SwerveHardware.VelocitySensor driveVel = new SwerveHardware.EncoderVelocitySensor(
 *           drive::getVelocity,                     // ticks per second
 *           SwerveConstants.DRIVE_TICKS_PER_REV,
 *           SwerveConstants.DRIVE_GEAR_RATIO,
 *           SwerveConstants.WHEEL_CIRCUMFERENCE_M);
 *
 *   AnalogInput absEnc = hardwareMap.get(AnalogInput.class, "flAbs");
 *   SwerveHardware.AngleSensor steerAngle = new SwerveHardware.AnalogAbsoluteAngleSensor(
 *           absEnc::getVoltage, absEnc::getMaxVoltage, FL_OFFSET_RAD, false);
 */
public final class SwerveHardware {

    private SwerveHardware() {
    }

    /** Anything that accepts an open-loop command in [-1, 1]. */
    public interface MotorOutput {
        void set(double power);
    }

    /** Azimuth position of a module, in radians. Absolute is strongly preferred. */
    public interface AngleSensor {
        double getAngleRadians();
    }

    /** Ground speed of a drive wheel, in metres per second. */
    public interface VelocitySensor {
        double getVelocityMetersPerSec();
    }

    /** Robot heading. Needed only for field-centric driving and heading hold. */
    public interface Gyro {
        double getYawRadians();

        double getYawRateRadiansPerSec();
    }

    /** Java 8 friendly stand-in for DoubleSupplier. */
    public interface DoubleSource {
        double get();
    }

    // ------------------------------------------------------------------
    // adapters
    // ------------------------------------------------------------------

    /**
     * Converts drive encoder ticks-per-second into metres per second.
     *
     * @param ticksPerSecond raw encoder velocity (FTC: DcMotorEx.getVelocity())
     * @param ticksPerRev    encoder counts per revolution of the motor output shaft
     * @param gearRatio      extra reduction between motor output shaft and wheel
     *                       (motor revs per wheel rev; 1.0 if direct)
     * @param wheelCircumferenceMeters  pi * wheel diameter
     */
    public static class EncoderVelocitySensor implements VelocitySensor {
        private final DoubleSource ticksPerSecond;
        private final double metersPerTick;

        public EncoderVelocitySensor(DoubleSource ticksPerSecond,
                                     double ticksPerRev,
                                     double gearRatio,
                                     double wheelCircumferenceMeters) {
            this.ticksPerSecond = ticksPerSecond;
            this.metersPerTick = wheelCircumferenceMeters / (ticksPerRev * gearRatio);
        }

        @Override
        public double getVelocityMetersPerSec() {
            return ticksPerSecond.get() * metersPerTick;
        }
    }

    /**
     * Turns a relative (quadrature) encoder into an azimuth AngleSensor.
     *
     * Caveat worth taking seriously: this is relative. It only knows where the
     * module is if you tell it. Either physically align every wheel forward
     * before enabling and call zero(), or seed it once at startup from an
     * absolute sensor via setCurrentAngle(). If you skip this, the robot will
     * happily drive off in a direction you did not ask for.
     */
    public static class EncoderAngleSensor implements AngleSensor {
        private final DoubleSource ticks;
        private final double radiansPerTick;
        private double offsetRadians = 0.0;

        /**
         * @param ticksPerRev      encoder counts per revolution of the steering
         *                         motor's output shaft
         * @param azimuthGearRatio steering motor output revs per full 360 degree
         *                         rotation of the module
         */
        public EncoderAngleSensor(DoubleSource ticks, double ticksPerRev, double azimuthGearRatio) {
            this.ticks = ticks;
            this.radiansPerTick = (2.0 * Math.PI) / (ticksPerRev * azimuthGearRatio);
        }

        @Override
        public double getAngleRadians() {
            return normalize(ticks.get() * radiansPerTick - offsetRadians);
        }

        /** Call with the wheel physically pointed straight ahead. */
        public void zero() {
            setCurrentAngle(0.0);
        }

        /** Declare the module's true current angle, e.g. read from an absolute sensor. */
        public void setCurrentAngle(double trueAngleRadians) {
            offsetRadians = ticks.get() * radiansPerTick - trueAngleRadians;
        }
    }

    /**
     * Absolute angle from an analog sensor whose voltage sweeps once per module
     * revolution (magnetic absolute encoder, Axon-style potentiometer feedback).
     * This is what you want on a heavy robot: no boot alignment ritual, and the
     * module cannot lose its place after a hard hit.
     *
     * @param offsetRadians the reading you see when the wheel points straight ahead,
     *                      measured once and hard-coded per module
     */
    public static class AnalogAbsoluteAngleSensor implements AngleSensor {
        private final DoubleSource voltage;
        private final DoubleSource maxVoltage;
        private final double offsetRadians;
        private final boolean inverted;

        public AnalogAbsoluteAngleSensor(DoubleSource voltage,
                                         DoubleSource maxVoltage,
                                         double offsetRadians,
                                         boolean inverted) {
            this.voltage = voltage;
            this.maxVoltage = maxVoltage;
            this.offsetRadians = offsetRadians;
            this.inverted = inverted;
        }

        @Override
        public double getAngleRadians() {
            double fraction = voltage.get() / Math.max(maxVoltage.get(), 1e-6);
            double angle = fraction * 2.0 * Math.PI;
            if (inverted) {
                angle = -angle;
            }
            return normalize(angle - offsetRadians);
        }
    }

    /** Fold any angle into [-pi, pi). */
    public static double normalize(double radians) {
        double r = radians % (2.0 * Math.PI);
        if (r >= Math.PI) {
            r -= 2.0 * Math.PI;
        } else if (r < -Math.PI) {
            r += 2.0 * Math.PI;
        }
        return r;
    }
}
