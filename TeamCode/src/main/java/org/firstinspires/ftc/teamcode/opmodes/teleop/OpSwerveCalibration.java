package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.swerve.SwerveRig;
import org.firstinspires.ftc.teamcode.swerve.WheelAngleStore;

/**
 * Sets "straight forward" for every wheel, by hand.
 *
 * ALL MOTORS ARE OFF in this OpMode and set to coast, so every wheel can be
 * turned by hand. Works in INIT and after PLAY alike.
 *
 *   1. Turn all four wheels so they point straight forward. Use a straight
 *      edge along the wheels if you can; every degree off here is a degree off
 *      every time the robot homes.
 *   2. Let go of the wheels and press TRIANGLE (Y).
 *   3. Straight away, every wheel's angle is saved as 0° (WheelAngleStore).
 *      This is what OpSwerve homes back to at every INIT. From here on the
 *      steering motor encoders follow the wheels, also when you turn them by
 *      hand in this OpMode, and keep the saved angles up to date.
 *   4. Keep everything still while each Through Bore is also averaged over its
 *      full sample count (the progress shows on telemetry); its offsets are
 *      then set so that position reads 0° too, and saved.
 *
 * CROSS (A) cancels a capture in progress. Press TRIANGLE again any time to
 * redo it.
 */
@TeleOp(name = "OpSwerveCalibration", group = "drive")
public class OpSwerveCalibration extends LinearOpMode {

    private SwerveRig rig;
    private final WheelAngleStore store = new WheelAngleStore();
    private String wheelSource = "";
    private boolean capturing = false;
    private boolean prevTriangle = false;
    private String message = "Turn all wheels straight forward by hand, then press TRIANGLE.";
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime rateTimer = new ElapsedTime();
    private int samplesSinceRate = 0;
    private double sampleRateHz = 0.0;

    @Override
    public void runOpMode() {
        rig = new SwerveRig(hardwareMap);
        rig.floatAllMotors();

        // If the wheel angles were saved before, start from them so turning the
        // wheels by hand here keeps the saved angles right.
        double[] saved = store.load();
        if (saved != null) {
            rig.seedFromAngles(saved);
            wheelSource = "following the wheels from the saved angles";
        } else {
            wheelSource = "no saved wheel angles yet: press TRIANGLE with the wheels straight";
        }

        try {
            while (opModeInInit()) {
                step();
            }
            while (opModeIsActive()) {
                step();
            }
        } finally {
            if (rig.isSeeded()) {
                rig.sampleEncoders();   // fresh hub read, so the last save is exact
                store.saveNow(rig.getWheelAnglesDegrees());
            }
            store.close();
        }
    }

    /** While capturing, sample the Through Bores this long per loop, ms. */
    private static final double CAPTURE_BURST_MS = 25.0;

    /** Encoder readings this close to 0°/360° can't be averaged reliably. */
    private static final double WRAP_MARGIN_DEG = 10.0;

    /**
     * Near its 0°/360° point the encoder's pulse flips between shortest and
     * longest, so an average there lands anywhere in between. Forward must not
     * sit there, or every homing starts from a wrong angle.
     */
    private String nearWrapWarning() {
        StringBuilder bad = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            double enc = rig.getOffsetDegrees(i) / -SwerveRig.ENCODER_DIRECTION;   // the captured encoder angle
            double fromWrap = Math.min(Math.abs(enc), 360.0 - Math.abs(enc));
            if (fromWrap < WRAP_MARGIN_DEG) {
                bad.append(' ').append(SwerveRig.LABELS[i]);
            }
        }
        return bad.length() == 0 ? ""
                : " WARNING:" + bad + " forward is within " + WRAP_MARGIN_DEG
                  + "° of its encoder's 0/360 point and will home unreliably. Re-seat that encoder"
                  + " on its shaft one hex flat (60°) round, then capture again.";
    }

    /** One pass of the loop; the same in INIT and after PLAY. */
    private void step() {
        if (capturing) {
            // Sample in bursts so the long capture average fills quickly.
            samplesSinceRate += rig.sampleEncodersFor(CAPTURE_BURST_MS);
        } else {
            rig.sampleEncoders();
            samplesSinceRate++;
        }

        // TRIANGLE: the wheels are straight right now. Save that as 0° at once,
        // then also start a fresh Through Bore average of this position.
        if (gamepad1.y && !prevTriangle) {
            double[] zero = {0.0, 0.0, 0.0, 0.0};
            rig.seedFromAngles(zero);
            wheelSource = store.saveNow(zero)
                    ? "wheels set to 0° and SAVED - OpSwerve homes to this position"
                    : "wheels set to 0° but " + store.getStatus();
            rig.restartSeedAverage();
            capturing = true;
            message = "CAPTURING - don't touch the wheels.";
        }
        prevTriangle = gamepad1.y;

        // Keep the saved angles following the wheels as they're turned by hand.
        if (rig.isSeeded()) {
            store.saveIfChanged(rig.getWheelAnglesDegrees());
        }

        if (gamepad1.a && capturing) {
            capturing = false;
            message = "Capture cancelled. Press TRIANGLE to start again.";
        }

        if (capturing && rig.seedAveragesFull()) {
            capturing = false;
            message = rig.captureForwardOffsets()
                    ? "Through Bore offsets captured and saved too."
                    : "Through Bore offsets captured but " + rig.getSaveStatus()
                      + ". Type them into DEFAULT_OFFSET_DEG in SwerveRig.java.";
            message += nearWrapWarning();
        }

        // Telemetry is slow; only send it a few times a second so the loop
        // spends its time sampling.
        if (telemetryTimer.milliseconds() < 200) {
            return;
        }
        sampleRateHz = samplesSinceRate / rateTimer.seconds();
        samplesSinceRate = 0;
        rateTimer.reset();
        telemetryTimer.reset();

        telemetry.addLine("MOTORS OFF. TRIANGLE = wheels are straight now | CROSS = cancel average");
        telemetry.addLine(message);
        telemetry.addData("wheel angles", wheelSource);
        if (rig.isSeeded()) {
            double[] now = rig.getWheelAnglesDegrees();
            telemetry.addData("wheels now", "FL %.1f°  FR %.1f°  BL %.1f°  BR %.1f°",
                    now[0], now[1], now[2], now[3]);
        }
        telemetry.addData("wheel angles file", store.getStatus());
        if (capturing) {
            telemetry.addLine(String.format("progress  FL %d  FR %d  BL %d  BR %d  / %d samples",
                    rig.getSeedSampleCount(0), rig.getSeedSampleCount(1),
                    rig.getSeedSampleCount(2), rig.getSeedSampleCount(3),
                    SwerveRig.SEED_SAMPLES));
        }
        telemetry.addData("offsets", rig.offsetsAsCode());
        telemetry.addData("offsets source", rig.getLoadStatus());
        telemetry.addData("saved", rig.getSaveStatus());
        rig.addThroughBoreTelemetry(telemetry, true, sampleRateHz);
        telemetry.update();
    }
}
