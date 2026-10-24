package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Driver 2's automatic drive sequences, run from OpSwerve:
 *
 *   CROSS    1. strafe LEFT   2. diagonal (angle editable)
 *   SQUARE   1. strafe RIGHT  2. diagonal (angle editable, 45° back-right to start with)
 *   TRIANGLE 1. strafe RIGHT  2. turn on the spot (LEFT if positive)
 *
 * Diagonal distances are measured along the diagonal itself. Times come from
 * the measured speeds in TimedMover. The heading is zeroed at the start and
 * held for the drives.
 *
 * Gamepad 2 (PS5 names, as on telemetry):
 *   CROSS / SQUARE / TRIANGLE  start that sequence    CIRCLE  stop at once
 *   D-pad up/down  choose a value     D-pad left/right  change it a little
 *   L1 / R1        change it a lot
 * Every change is saved on the robot straight away (SwerveSequence.txt).
 * Driver 1 moving a stick or the D-pad also stops it (see OpSwerve).
 */
public class DriveSequence {

    private static final String FILE_NAME = "SwerveSequence.txt";

    // ---------------- values, in the order they're saved ----------------
    // (new ones are added at the end, so an older file still loads)

    private static final int X_STRAFE = 0;
    private static final int X_DIAGONAL = 1;
    private static final int X_ANGLE = 2;
    private static final int SQ_DIAGONAL = 3;
    private static final int SQ_STRAFE = 4;
    private static final int SQ_ANGLE = 5;
    private static final int TR_STRAFE = 6;
    private static final int TR_TURN = 7;

    private static final String[] NAMES = {
            "✕ 1. strafe LEFT distance",
            "✕ 2. diagonal distance",
            "✕ 2. diagonal angle",
            "□ 2. diagonal distance",
            "□ 1. strafe RIGHT distance",
            "□ 2. diagonal angle",
            "△ 1. strafe RIGHT distance",
            "△ 2. turn",
    };
    private static final boolean[] IS_ANGLE = {false, false, true, false, false, true, false, true};
    private static final double[] DEFAULTS = {3.0, 8.0, 30.0, 3.0, 3.0, -135.0, 4.0, 45.0};
    /** D-pad left/right step, and bumper step. */
    private static final double[] SMALL_STEP = {0.05, 0.05, 1.0, 0.05, 0.05, 1.0, 0.05, 1.0};
    private static final double[] BIG_STEP = {0.5, 0.5, 5.0, 0.5, 0.5, 5.0, 0.5, 5.0};
    private static final double[] MIN = {0.0, 0.0, -180.0, 0.0, 0.0, -180.0, 0.0, -180.0};
    private static final double[] MAX = {30.0, 30.0, 180.0, 30.0, 30.0, 180.0, 30.0, 180.0};

    /** Order shown on telemetry and walked through with the D-pad: sequence by sequence. */
    private static final int[] SHOWN = {X_STRAFE, X_DIAGONAL, X_ANGLE, SQ_STRAFE, SQ_DIAGONAL, SQ_ANGLE, TR_STRAFE, TR_TURN};

    // ---------------- sequences ----------------

    private static final int SEQ_CROSS = 0;
    private static final int SEQ_SQUARE = 1;
    private static final int SEQ_TRIANGLE = 2;
    private static final String[] SEQ_NAMES = {"✕ CROSS", "□ SQUARE", "△ TRIANGLE"};

    /** Direction of travel, TimedMover style: degrees from forward, + is left. */
    private static final double LEFT_DEG = 90.0;
    private static final double RIGHT_DEG = -90.0;

    private final SwerveRig rig;
    private final TimedMover mover;

    private final double[] values = DEFAULTS.clone();
    /** Index into SHOWN. */
    private int selected = 0;
    private String saveStatus;

    private boolean running = false;
    private int sequence = SEQ_CROSS;
    private int step = 0;
    private double totalTime = 0.0;
    private String status = "ready";

    private boolean prevA, prevB, prevX, prevY, prevUp, prevDown, prevLeft, prevRight, prevLb, prevRb;

    public DriveSequence(SwerveRig rig, HardwareMap hardwareMap) {
        this.rig = rig;
        this.mover = new TimedMover(rig, hardwareMap);
        load();
    }

    public boolean isRunning() {
        return running;
    }

    /** Stop at once, e.g. because driver 1 took over. */
    public void stop(String why) {
        if (running) {
            mover.stop();
            running = false;
            status = "STOPPED in " + SEQ_NAMES[sequence] + " step " + (step + 1) + ": " + why;
        }
    }

    /** Call once per loop, after rig.sampleEncoders(). */
    public void update(Gamepad gamepad, double dt) {
        boolean a = gamepad.a;
        boolean b = gamepad.b;
        boolean x = gamepad.x;
        boolean y = gamepad.y;
        boolean up = gamepad.dpad_up;
        boolean down = gamepad.dpad_down;
        boolean left = gamepad.dpad_left;
        boolean right = gamepad.dpad_right;
        boolean lb = gamepad.left_bumper;
        boolean rb = gamepad.right_bumper;

        if (running) {
            if (b && !prevB) {
                stop("driver 2 pressed CIRCLE");
            }
        } else {
            int field = SHOWN[selected];
            if (a && !prevA) {
                start(SEQ_CROSS);
            } else if (x && !prevX) {
                start(SEQ_SQUARE);
            } else if (y && !prevY) {
                start(SEQ_TRIANGLE);
            } else if (up && !prevUp) {
                selected = (selected + SHOWN.length - 1) % SHOWN.length;
            } else if (down && !prevDown) {
                selected = (selected + 1) % SHOWN.length;
            } else if (right && !prevRight) {
                change(field, SMALL_STEP[field]);
            } else if (left && !prevLeft) {
                change(field, -SMALL_STEP[field]);
            } else if (rb && !prevRb) {
                change(field, BIG_STEP[field]);
            } else if (lb && !prevLb) {
                change(field, -BIG_STEP[field]);
            }
        }

        prevA = a;
        prevB = b;
        prevX = x;
        prevY = y;
        prevUp = up;
        prevDown = down;
        prevLeft = left;
        prevRight = right;
        prevLb = lb;
        prevRb = rb;

        mover.update(dt);
        if (running && mover.consumeFinished()) {
            step++;
            if (step < 2) {
                startStep();
            } else {
                running = false;
                status = String.format("%s DONE in %.1f s, heading %.1f°",
                        SEQ_NAMES[sequence], totalTime, rig.getHeadingDegrees());
            }
        }
        if (running) {
            totalTime += dt;
        }
    }

    private void change(int field, double delta) {
        double v = values[field] + delta;
        // Round away float error from the steps (0.05 m, 1°).
        v = Math.round(v * 100.0) / 100.0;
        values[field] = Math.max(MIN[field], Math.min(MAX[field], v));
        save();
    }

    private void start(int seq) {
        rig.resetHeading();
        sequence = seq;
        running = true;
        step = 0;
        totalTime = 0.0;
        startStep();
    }

    private void startStep() {
        status = "running";
        if (sequence == SEQ_TRIANGLE && step == 1) {
            mover.startTurn(values[TR_TURN]);
            return;
        }
        double dir = stepDirection(sequence, step);
        mover.startDrive(dir, TimedMover.cruiseSecondsFor(stepDistance(sequence, step), dir), 0.0);
    }

    /** Direction of travel of a drive step, TimedMover style: degrees, + is left. */
    private double stepDirection(int seq, int s) {
        switch (seq) {
            case SEQ_CROSS:
                return s == 0 ? LEFT_DEG : values[X_ANGLE];
            case SEQ_SQUARE:
                return s == 0 ? RIGHT_DEG : values[SQ_ANGLE];
            default:
                return RIGHT_DEG;
        }
    }

    /** Distance of a drive step, metres. */
    private double stepDistance(int seq, int s) {
        switch (seq) {
            case SEQ_CROSS:
                return s == 0 ? values[X_STRAFE] : values[X_DIAGONAL];
            case SEQ_SQUARE:
                return s == 0 ? values[SQ_STRAFE] : values[SQ_DIAGONAL];
            default:
                return values[TR_STRAFE];
        }
    }

    // ------------------------------------------------------------------
    // saving
    // ------------------------------------------------------------------

    private File file() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    private void load() {
        File f = file();
        if (!f.exists()) {
            saveStatus = "defaults (nothing saved yet)";
            return;
        }
        try {
            String[] parts = ReadWriteFile.readFileOrThrow(f).trim().split("\\s+");
            if (parts.length > values.length) {
                throw new IllegalArgumentException("expected at most " + values.length + " numbers");
            }
            // A file from before a value was added keeps that value's default.
            for (int i = 0; i < parts.length; i++) {
                values[i] = Double.parseDouble(parts[i]);
            }
            saveStatus = "loaded from " + FILE_NAME;
        } catch (Exception e) {
            saveStatus = "defaults (couldn't read " + FILE_NAME + ": " + e.getMessage() + ")";
        }
    }

    private void save() {
        StringBuilder text = new StringBuilder();
        for (double v : values) {
            text.append(v).append('\n');
        }
        try {
            ReadWriteFile.writeFileOrThrow(file(), text.toString());
            saveStatus = "SAVED";
        } catch (Exception e) {
            saveStatus = "SAVE FAILED: " + e.getMessage();
        }
    }

    // ------------------------------------------------------------------
    // telemetry
    // ------------------------------------------------------------------

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("=== DRIVER 2: ✕ CROSS | □ SQUARE | △ TRIANGLE start | ○ CIRCLE stop ===");
        if (running) {
            telemetry.addLine(String.format("%s step %d/2 %s: %s %.1f s",
                    SEQ_NAMES[sequence], step + 1, stepText(sequence, step),
                    mover.getPhase(), mover.getPhaseTime()));
            telemetry.addLine(String.format("heading %.1f°, drive power %.2f",
                    rig.getHeadingDegrees(), mover.getDrivePower()));
            return;
        }
        telemetry.addLine("last run: " + status);
        for (int k = 0; k < SHOWN.length; k++) {
            int i = SHOWN[k];
            String value = IS_ANGLE[i]
                    ? (i == TR_TURN ? turnText(values[i]) : angleText(values[i]))
                    : String.format("%.2f m", values[i]);
            telemetry.addLine((k == selected ? ">> " : "    ") + NAMES[i] + ": " + value);
        }
        int field = SHOWN[selected];
        telemetry.addLine(String.format("CHANGING: %s | D-pad ◀ ▶ ±%s, L1 / R1 ±%s",
                NAMES[field], amountText(SMALL_STEP[field], field), amountText(BIG_STEP[field], field)));
        telemetry.addLine("D-pad ▲ ▼ choose value | saved automatically on every change: " + saveStatus);
        for (int seq = SEQ_CROSS; seq <= SEQ_TRIANGLE; seq++) {
            telemetry.addLine("plan " + SEQ_NAMES[seq] + ": " + stepText(seq, 0) + " then " + stepText(seq, 1));
        }
    }

    /** What a step does, e.g. "strafe right 4.00 m (8.6 s cruise)". */
    private String stepText(int seq, int s) {
        if (seq == SEQ_TRIANGLE && s == 1) {
            return turnText(values[TR_TURN]);
        }
        double dir = stepDirection(seq, s);
        double dist = stepDistance(seq, s);
        String what = s == 0 ? (dir > 0 ? "strafe left" : "strafe right") : "diagonal " + angleText(dir);
        return String.format("%s %.2f m (%.1f s cruise)", what, dist, TimedMover.cruiseSecondsFor(dist, dir));
    }

    private static String angleText(double deg) {
        if (deg == 0.0) {
            return "0° (straight forward)";
        }
        return String.format("%.0f° %s of forward", Math.abs(deg), deg > 0 ? "LEFT" : "RIGHT");
    }

    private static String turnText(double deg) {
        return String.format("turn %.0f° %s", Math.abs(deg), deg >= 0 ? "LEFT" : "RIGHT");
    }

    private static String amountText(double amount, int field) {
        return IS_ANGLE[field] ? String.format("%.0f°", amount) : String.format("%.2f m", amount);
    }
}
