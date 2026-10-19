package org.firstinspires.ftc.teamcode.pedropathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.swerve.SwerveConstants;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.SwerveRobot;

/**
 * Lets Pedro Pathing follow paths on our swerve drive.
 *
 * Pedro hands every drivetrain three power vectors in the FIELD frame, each at
 * most maxPowerScaling long:
 *   corrective - push back onto the path (highest priority)
 *   heading    - turn power, encoded as a vector along the robot's heading,
 *                positive = counter-clockwise
 *   pathing    - drive along the path (gets whatever power is left)
 *
 * We keep Pedro's priority order, rotate the translation into the robot frame,
 * and hand SwerveDrive a chassis request in m/s and rad/s. SwerveDrive then does
 * the inverse kinematics, accel limiting and desaturation as usual.
 *
 * Heading hold is switched off here: Pedro runs its own heading controller.
 */
public class PedroSwerveDrivetrain extends Drivetrain {

    private static final double METERS_PER_INCH = 0.0254;

    private final SwerveRobot robot;
    private final SwerveDrive drive;
    private final VoltageSensor voltageSensor;

    /** Free-roll speeds Pedro uses for its braking model, inches per second. */
    private double xVelocity = SwerveConstants.MAX_SPEED_MPS / METERS_PER_INCH;
    private double yVelocity = SwerveConstants.MAX_SPEED_MPS / METERS_PER_INCH;

    public PedroSwerveDrivetrain(HardwareMap hardwareMap, SwerveRobot robot) {
        this.robot = robot;
        this.drive = robot.getDrive();
        this.drive.setHeadingHold(false);
        this.voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.maxPowerScaling = 1.0;
        this.nominalVoltage = 12.0;
    }

    /**
     * @return {forward, left, counter-clockwise} as fractions of full speed,
     *         in the robot frame
     */
    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower,
                                   Vector pathingPower, double robotHeading) {
        double max = maxPowerScaling;

        double cx = correctivePower.getXComponent();
        double cy = correctivePower.getYComponent();
        double cMag = Math.hypot(cx, cy);
        if (cMag > max) {
            cx *= max / cMag;
            cy *= max / cMag;
            cMag = max;
        }

        // Signed turn power: the heading vector's length along the robot heading.
        double turn = headingPower.getXComponent() * Math.cos(robotHeading)
                + headingPower.getYComponent() * Math.sin(robotHeading);

        double tx = cx;
        double ty = cy;
        if (cMag >= max) {
            // Correction takes everything.
            turn = 0.0;
        } else {
            // Turning gets what correction leaves, pathing gets the rest.
            double room = max - cMag;
            turn = Math.max(-room, Math.min(room, turn));
            double budget = max - Math.abs(turn);

            double px = pathingPower.getXComponent();
            double py = pathingPower.getYComponent();
            double scale = pathingScale(cx, cy, px, py, budget);
            tx = cx + px * scale;
            ty = cy + py * scale;
        }

        // Field frame to robot frame.
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double forward = tx * cos - ty * sin;
        double left = tx * sin + ty * cos;

        return new double[]{forward, left, turn};
    }

    /**
     * Largest s in [0, 1] with |c + s p| <= budget. c is already inside the
     * budget, so s = 0 always fits.
     */
    private static double pathingScale(double cx, double cy, double px, double py, double budget) {
        if (Math.hypot(cx + px, cy + py) <= budget) {
            return 1.0;
        }
        // Solve |c + s p|^2 = budget^2 for the positive root.
        double a = px * px + py * py;
        double b = 2.0 * (cx * px + cy * py);
        double c = cx * cx + cy * cy - budget * budget;
        if (a < 1e-12) {
            return 0.0;
        }
        double disc = Math.max(0.0, b * b - 4.0 * a * c);
        double s = (-b + Math.sqrt(disc)) / (2.0 * a);
        return Math.max(0.0, Math.min(1.0, s));
    }

    @Override
    public void runDrive(double[] drivePowers) {
        robot.clearBulkCache();
        drive.drive(
                drivePowers[0] * SwerveConstants.MAX_SPEED_MPS,
                drivePowers[1] * SwerveConstants.MAX_SPEED_MPS,
                drivePowers[2] * SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
                false);
    }

    @Override
    public void updateConstants() {
    }

    @Override
    public void breakFollowing() {
        drive.stop();
    }

    @Override
    public void startTeleopDrive() {
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        // Swerve motors are always in BRAKE; see SwerveRobot.
    }

    @Override
    public double xVelocity() {
        return xVelocity;
    }

    @Override
    public double yVelocity() {
        return yVelocity;
    }

    @Override
    public void setXVelocity(double xMovement) {
        xVelocity = xMovement;
    }

    @Override
    public void setYVelocity(double yMovement) {
        yVelocity = yMovement;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    @Override
    public String debugString() {
        return "PedroSwerveDrivetrain{maxPowerScaling=" + maxPowerScaling
                + ", xVelocity=" + xVelocity + ", yVelocity=" + yVelocity + "}";
    }
}
