package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.IntakeIndex;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.swerve.SwerveRobot;

/**
 * Competition TeleOp: swerve drive plus the intake and outtake.
 *
 * INIT: align all four wheels forward, press A. The drive stays disabled until
 * the wheels are homed; the mechanisms work either way.
 *
 * gamepad1
 *   left stick       drive (field-centric)        right stick X   turn
 *   right bumper     hold for robot-centric       left bumper     X-lock wheels
 *   back             reset field heading          right trigger   intake + index
 *   cross/triangle   slides down/up               circle          toggle flywheel speed
 *   dpad left/right  barrier
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    private SwerveRobot robot;
    private SwerveDrive drive;
    private Outtake outtake;
    private IntakeIndex intake;

    @Override
    public void init() {
        robot = new SwerveRobot(hardwareMap);
        drive = robot.getDrive();
        outtake = new Outtake(hardwareMap);
        intake = new IntakeIndex(hardwareMap);
    }

    @Override
    public void init_loop() {
        robot.clearBulkCache();
        if (gamepad1.a) {
            robot.home();
        }
        robot.addHomingTelemetry(telemetry);
    }

    @Override
    public void start() {
        drive.reset();
    }

    @Override
    public void loop() {
        robot.clearBulkCache();

        if (!robot.isHomed()) {
            // An unhomed swerve drive picks its own direction, so it stays still.
            drive.stop();
            telemetry.addLine("DRIVE DISABLED - wheels not homed. Restart and press A in init.");
        } else if (gamepad1.left_bumper) {
            drive.lockWheels();
        } else {
            // Sticks report y-down and x-right; negate both for +x forward, +y left.
            drive.driveNormalized(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    !gamepad1.right_bumper);
        }

        if (gamepad1.back) {
            robot.resetYaw();
        }

        outtake.update(gamepad1);
        intake.activateDeactivate(gamepad1.right_trigger);

        robot.addModuleTelemetry(telemetry);
    }

    @Override
    public void stop() {
        drive.stop();
    }
}
