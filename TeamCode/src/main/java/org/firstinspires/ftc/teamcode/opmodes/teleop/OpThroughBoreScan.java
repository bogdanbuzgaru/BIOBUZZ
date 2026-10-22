package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Finds which digital channels really get a REV Through Bore signal.
 *
 * Reads EVERY digital device in the active robot configuration, not just the
 * four swerve encoders, and shows for each one how often it reads high and
 * how often it changes. No motors move.
 *
 *   PULSING    the absolute (PWM) signal arrives here. "% high" is the angle
 *              / 360, so it changes when you turn that wheel by hand.
 *   ALWAYS HIGH / ALWAYS LOW   nothing arrives on this channel.
 *
 * Each digital port on a REV hub carries TWO channels (0-1, 2-3, 4-5, 6-7).
 * To find a signal that's on the other channel of a port, add a digital
 * device on every channel 0..7 of both hubs to a test configuration (any
 * names) and run this.
 */
@TeleOp(name = "OpThroughBoreScan", group = "drive")
public class OpThroughBoreScan extends LinearOpMode {

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        List<DigitalChannel> channels = hardwareMap.getAll(DigitalChannel.class);
        List<String> names = new ArrayList<>();
        for (DigitalChannel c : channels) {
            c.setMode(DigitalChannel.Mode.INPUT);
            names.add(String.join("/", hardwareMap.getNamesOf(c)));
        }
        int n = channels.size();
        long[] samples = new long[n];
        long[] highs = new long[n];
        long[] toggles = new long[n];
        boolean[] last = new boolean[n];

        ElapsedTime window = new ElapsedTime();
        while (opModeInInit() || opModeIsActive()) {
            for (LynxModule hub : hubs) {
                hub.clearBulkCache();
            }
            for (int i = 0; i < n; i++) {
                boolean high = channels.get(i).getState();
                if (samples[i] > 0 && high != last[i]) {
                    toggles[i]++;
                }
                last[i] = high;
                samples[i]++;
                if (high) {
                    highs[i]++;
                }
            }

            // Show and restart every second.
            if (window.seconds() < 1.0) {
                continue;
            }
            telemetry.addLine("Turn a wheel by hand: its channel's % high should follow.");
            telemetry.addLine("name | port | % high | changes/s | verdict");
            if (n == 0) {
                telemetry.addLine("No digital devices in this configuration.");
            }
            for (int i = 0; i < n; i++) {
                double pct = samples[i] == 0 ? 0.0 : 100.0 * highs[i] / samples[i];
                String verdict = toggles[i] > 0 ? "PULSING (" + String.format("%.0f°", pct * 3.6) + ")"
                        : (highs[i] > 0 ? "ALWAYS HIGH" : "ALWAYS LOW");
                telemetry.addData(names.get(i), "%s | %5.1f%% | %.0f | %s",
                        channels.get(i).getConnectionInfo(), pct,
                        toggles[i] / window.seconds(), verdict);
                samples[i] = 0;
                highs[i] = 0;
                toggles[i] = 0;
            }
            telemetry.update();
            window.reset();
        }
    }
}
