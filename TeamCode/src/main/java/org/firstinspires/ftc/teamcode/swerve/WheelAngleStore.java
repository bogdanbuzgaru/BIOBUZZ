package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Remembers where each wheel points between runs, in a file on the robot
 * (FIRST/settings/SwerveWheelAngles.txt, four angles in degrees, FL FR BL BR).
 *
 * The steering motor encoders know the wheel angles exactly while the robot is
 * on, but their count restarts at 0 every INIT. Saving the angles while
 * running and reading them back at the next INIT carries that knowledge over,
 * so OpSwerve can turn every wheel back to 0 degrees.
 *
 * It's only as good as the assumption that nobody turns the wheels while the
 * robot is off. If someone does, set the wheels straight and press TRIANGLE in
 * OpSwerveCalibration to start again from 0.
 *
 * Writes happen on a background thread so the control loop never waits on the
 * Control Hub's storage.
 */
public class WheelAngleStore {

    private static final String FILE_NAME = "SwerveWheelAngles.txt";

    /** Save at most this often while running. */
    private static final double MIN_SAVE_INTERVAL_S = 0.25;
    /** Only save when some wheel has moved at least this much since the last save. */
    private static final double MIN_CHANGE_DEG = 0.5;

    private final ExecutorService writer = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "WheelAngleStore");
        t.setDaemon(true);
        return t;
    });
    private final AtomicBoolean writeInProgress = new AtomicBoolean(false);

    private double[] lastQueued = null;
    private long lastQueuedNanos = 0L;
    private volatile String status = "nothing saved yet this run";

    private static File file() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    /**
     * Reads the saved angles.
     *
     * @return four angles in degrees, or null if there's no usable file;
     *         getStatus() says which
     */
    public double[] load() {
        File f = file();
        if (!f.exists()) {
            status = "no saved wheel angles (" + f.getPath() + ")";
            return null;
        }
        try {
            String[] parts = ReadWriteFile.readFileOrThrow(f).trim().split("\\s+");
            if (parts.length != 4) {
                status = "saved wheel angles unreadable: expected 4 numbers, found " + parts.length;
                return null;
            }
            double[] deg = new double[4];
            for (int i = 0; i < 4; i++) {
                deg[i] = Double.parseDouble(parts[i]);
            }
            status = "loaded from " + f.getPath();
            return deg;
        } catch (Exception e) {
            status = "saved wheel angles unreadable: " + e.getMessage();
            return null;
        }
    }

    /**
     * Call every loop with the current wheel angles. Saves in the background
     * when they've changed, at most every MIN_SAVE_INTERVAL_S.
     */
    public void saveIfChanged(double[] deg) {
        long now = System.nanoTime();
        if (lastQueued != null) {
            if ((now - lastQueuedNanos) / 1e9 < MIN_SAVE_INTERVAL_S || !changed(deg)) {
                return;
            }
        }
        if (!writeInProgress.compareAndSet(false, true)) {
            return;   // previous write still going; try again next loop
        }
        final String content = format(deg);
        lastQueued = deg.clone();
        lastQueuedNanos = now;
        writer.submit(() -> {
            try {
                write(content);
            } finally {
                writeInProgress.set(false);
            }
        });
    }

    /**
     * Save right now and wait for it (after any background write already
     * queued). Use at STOP and when setting a new reference.
     *
     * @return false if the save failed; getStatus() says why
     */
    public boolean saveNow(double[] deg) {
        final String content = format(deg);
        lastQueued = deg.clone();
        lastQueuedNanos = System.nanoTime();
        try {
            Future<Boolean> done = writer.submit(() -> write(content));
            return done.get(2, TimeUnit.SECONDS);
        } catch (Exception e) {
            status = "SAVE FAILED: " + e.getMessage();
            return false;
        }
    }

    /** Finish any pending write and stop the background thread. */
    public void close() {
        writer.shutdown();
        try {
            writer.awaitTermination(2, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public String getStatus() {
        return status;
    }

    private boolean changed(double[] deg) {
        for (int i = 0; i < 4; i++) {
            double diff = Math.toDegrees(SwerveHardware.normalize(Math.toRadians(deg[i] - lastQueued[i])));
            if (Math.abs(diff) >= MIN_CHANGE_DEG) {
                return true;
            }
        }
        return false;
    }

    /** Writes the file and reads it back to make sure it really landed. */
    private boolean write(String content) {
        File f = file();
        try {
            ReadWriteFile.writeFileOrThrow(f, content);
            if (!ReadWriteFile.readFileOrThrow(f).equals(content)) {
                status = "SAVE FAILED: file reads back different";
                return false;
            }
            status = "saved to " + f.getPath();
            return true;
        } catch (Exception e) {
            status = "SAVE FAILED: " + e.getMessage();
            return false;
        }
    }

    private static String format(double[] deg) {
        return deg[0] + "\n" + deg[1] + "\n" + deg[2] + "\n" + deg[3] + "\n";
    }
}
