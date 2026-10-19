package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Bring-up and calibration OpMode. Run this BEFORE SwerveTeleOp, with the robot
 * on blocks.
 *
 * It shows every encoder live and nothing drives on its own, so it is safe to
 * turn modules by hand while watching the numbers.
 *
 * WHAT YOU CAN FIND HERE
 *
 *  1. AZIMUTH_INVERTED, per module. Turn a module counter-clockwise by hand
 *     (seen from above). Its angle must go UP. If it goes down, that module's
 *     flag is true. Get this wrong and the steering loop becomes positive
 *     feedback: the module spins to full speed and runs away. Check it here,
 *     on blocks.
 *
 *  2. DRIVE_INVERTED, per module. Hold RIGHT BUMPER to creep the selected
 *     module's drive motor forward. Its drive ticks must go UP and the wheel
 *     must turn as if driving the robot forward.
 *
 *  3. Scaling. Rotate a module exactly 90 degrees against a square. The
 *     continuous angle should read 90 plus or minus a degree or so. If it reads
 *     something like 266 (90 x 2.955), the Through Bore is reading the motor
 *     shaft rather than the azimuth axis and AZIMUTH_GEAR_RATIO is wrong.
 *
 *  4. Reference offsets. If aligning each wheel to "forward" by eye is awkward,
 *     park the modules against a jig or rail instead, then read off how far that
 *     reference sits from forward and paste the four numbers into
 *     SwerveConstants.AZIMUTH_OFFSET_DEG.
 *
 *  5. Noise. The drift column shows how much each reading has wandered while
 *     the module sat still. Anything over a degree means the encoder lead is
 *     picking up motor noise, and the fix is cable routing, not software.
 *
 * CONTROLS
 *   DPAD LEFT / RIGHT   select a module
 *   A                   zero ALL modules at their current position
 *   X                   zero ONLY the selected module
 *   Y                   capture the selected module's angle as a reference
 *                       offset, and print it for pasting
 *   B                   clear captured offsets and drift tracking
 *   LEFT BUMPER         creep the selected module's STEER motor (hands off)
 *   RIGHT BUMPER        creep the selected module's DRIVE motor (hands off)
 *   LEFT TRIGGER        hold to creep in reverse
 *
 * NOTE ON PERSISTENCE. On the quadrature path there is no offset that survives
 * a power cycle: the Control Hub zeroes the encoder count when the OpMode
 * starts, wherever the module happens to be sitting. The offsets you capture
 * here describe the angle between your chosen reference and forward, which is a
 * property of the robot, not of the encoder. You still align to that reference
 * each run. Persistent offsets only become possible with the absolute pulse
 * output and an OctoQuad.
 */
@TeleOp(name = "Swerve Calibration", group = "drive")
public class SwerveCalibration extends LinearOpMode {

    private static final String[] NAMES = {"fl", "fr", "bl", "br"};
    private static final double CREEP_POWER = 0.15;

    private final DcMotorEx[] driveMotors = new DcMotorEx[4];
    private final DcMotorEx[] steerMotors = new DcMotorEx[4];
    private final ThroughBoreSteeringEncoder[] azimuth = new ThroughBoreSteeringEncoder[4];

    private final double[] capturedOffsetDeg = new double[4];
    private final boolean[] captured = new boolean[4];
    private final double[] minSeen = new double[4];
    private final double[] maxSeen = new double[4];

    private List<LynxModule> hubs;
    private int selected = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (int i = 0; i < 4; i++) {
            driveMotors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i] + "Drive");
            steerMotors[i] = hardwareMap.get(DcMotorEx.class, NAMES[i] + "Steer");

            // Drive encoders are reset so the tick counts start from a known place.
            driveMotors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Steering encoders are deliberately NOT reset, so you can watch
            // what the hub already had and see the raw count move continuously.
            steerMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // FLOAT, not BRAKE: you want to be able to turn these by hand.
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            steerMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            final int k = i;
            // Deliberately NOT inverted here. This OpMode is how you discover
            // what the inversions should be, so it has to show the raw truth.
            azimuth[i] = new ThroughBoreSteeringEncoder(
                    () -> steerMotors[k].getCurrentPosition(),
                    SwerveConstants.AZIMUTH_GEAR_RATIO,
                    false);
            minSeen[i] = Double.POSITIVE_INFINITY;
            maxSeen[i] = Double.NEGATIVE_INFINITY;
        }

        telemetry.addLine("Robot ON BLOCKS. Press PLAY.");
        telemetry.update();
        waitForStart();

        ElapsedTime loopTimer = new ElapsedTime();
        boolean prevLeft = false, prevRight = false;
        boolean prevA = false, prevX = false, prevY = false, prevB = false;

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            // ---- module selection ----
            if (gamepad1.dpad_right && !prevRight) selected = (selected + 1) % 4;
            if (gamepad1.dpad_left && !prevLeft) selected = (selected + 3) % 4;
            prevRight = gamepad1.dpad_right;
            prevLeft = gamepad1.dpad_left;

            // ---- zeroing ----
            if (gamepad1.a && !prevA) {
                for (int i = 0; i < 4; i++) {
                    azimuth[i].home();
                    resetTracking(i);
                }
            }
            if (gamepad1.x && !prevX) {
                azimuth[selected].home();
                resetTracking(selected);
            }
            if (gamepad1.y && !prevY) {
                capturedOffsetDeg[selected] = azimuth[selected].getAngleDegrees();
                captured[selected] = true;
            }
            if (gamepad1.b && !prevB) {
                for (int i = 0; i < 4; i++) {
                    captured[i] = false;
                    resetTracking(i);
                }
            }
            prevA = gamepad1.a;
            prevX = gamepad1.x;
            prevY = gamepad1.y;
            prevB = gamepad1.b;

            // ---- creep, one motor at a time ----
            double sign = gamepad1.left_trigger > 0.5 ? -1.0 : 1.0;
            for (int i = 0; i < 4; i++) {
                steerMotors[i].setPower(
                        (i == selected && gamepad1.left_bumper) ? CREEP_POWER * sign : 0.0);
                driveMotors[i].setPower(
                        (i == selected && gamepad1.right_bumper) ? CREEP_POWER * sign : 0.0);
            }

            // ---- telemetry ----
            telemetry.addLine("DPAD select | A zero all | X zero one | Y capture | B clear");
            telemetry.addLine("LB creep steer | RB creep drive | LT reverse");
            telemetry.addData("loop", "%.1f ms   (keep under ~15)", loopMs);
            telemetry.addData("deg per tick", "%.4f", azimuth[0].getDegreesPerTick());
            telemetry.addLine();

            for (int i = 0; i < 4; i++) {
                double angle = azimuth[i].getAngleDegrees();
                double cont = azimuth[i].getContinuousAngleDegrees();
                if (angle < minSeen[i]) minSeen[i] = angle;
                if (angle > maxSeen[i]) maxSeen[i] = angle;

                telemetry.addLine((i == selected ? ">> " : "   ") + NAMES[i].toUpperCase());
                telemetry.addData("   azimuth", "%8.2f deg    raw %8.0f",
                        angle, azimuth[i].getRawTicks());
                telemetry.addData("   continuous", "%8.2f deg    drift %5.2f",
                        cont, maxSeen[i] - minSeen[i]);
                telemetry.addData("   drive ticks", "%d", driveMotors[i].getCurrentPosition());
                if (captured[i]) {
                    telemetry.addData("   CAPTURED", "%.2f deg", capturedOffsetDeg[i]);
                }
            }

            // ---- paste-ready output ----
            boolean anyCaptured = false;
            for (boolean c : captured) {
                anyCaptured |= c;
            }
            if (anyCaptured) {
                StringBuilder sb = new StringBuilder("{ ");
                for (int i = 0; i < 4; i++) {
                    sb.append(String.format("%.2f", captured[i] ? capturedOffsetDeg[i] : 0.0));
                    if (i < 3) sb.append(", ");
                }
                sb.append(" }");
                telemetry.addLine();
                telemetry.addLine("AZIMUTH_OFFSET_DEG = " + sb);
            }

            telemetry.update();
        }

        for (int i = 0; i < 4; i++) {
            steerMotors[i].setPower(0.0);
            driveMotors[i].setPower(0.0);
        }
    }

    private void resetTracking(int i) {
        minSeen[i] = Double.POSITIVE_INFINITY;
        maxSeen[i] = Double.NEGATIVE_INFINITY;
    }
}
