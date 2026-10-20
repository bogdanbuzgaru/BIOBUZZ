package org.firstinspires.ftc.teamcode.swerve;

/**
 * Module angle from the steering motor's own built-in encoder.
 *
 * The encoder sits on the motor, before the steering gears, so the angle is
 * motor ticks divided by (ticks per motor rev x steering gear ratio). It is
 * relative: setCurrentAngle() tells it where the wheel really points, after
 * which it tracks every turn. Gear backlash is invisible to it, so expect a
 * degree or two of error when the module changes direction.
 *
 * Set the steering motor's direction so positive power turns the module
 * counter-clockwise seen from above; the SDK flips the encoder sign with the
 * motor direction, so the ticks then rise as the angle rises.
 */
public class SteerMotorAngleSensor implements SwerveHardware.AngleSensor {

    private final SwerveHardware.DoubleSource ticks;
    private final double radiansPerTick;
    private final double sign;
    private double offsetRadians = 0.0;

    /**
     * @param ticks            e.g. steerMotor::getCurrentPosition
     * @param ticksPerMotorRev encoder counts per motor output shaft revolution
     * @param steerGearRatio   motor output revs per 360 degrees of module rotation
     * @param reversed         true if the count FALLS when the motor gets positive
     *                         power (a miswired or reversed encoder). Wrong here
     *                         means positive feedback: the module spins away.
     */
    public SteerMotorAngleSensor(SwerveHardware.DoubleSource ticks,
                                 double ticksPerMotorRev,
                                 double steerGearRatio,
                                 boolean reversed) {
        this.ticks = ticks;
        this.radiansPerTick = (2.0 * Math.PI) / (ticksPerMotorRev * steerGearRatio);
        this.sign = reversed ? -1.0 : 1.0;
    }

    @Override
    public double getAngleRadians() {
        return SwerveHardware.normalize(getTicks() * radiansPerTick + offsetRadians);
    }

    /** Declare where the wheel really points right now, in radians. */
    public void setCurrentAngle(double trueAngleRadians) {
        offsetRadians = trueAngleRadians - getTicks() * radiansPerTick;
    }

    /** Encoder count with the reversal applied: rises as the angle rises. */
    public double getTicks() {
        return sign * ticks.get();
    }
}
