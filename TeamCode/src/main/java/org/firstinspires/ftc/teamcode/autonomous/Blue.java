package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeIndex;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.statemachine.Statemachine;

import java.io.File;

@Autonomous
public class Blue extends OpMode {
    public enum AutoState {
        SHOOT_FIRST,
        TAKE_BALLS,
        GO_SHOOT,
        PARK
    }
    private Follower follower;
    private Paths paths;
    private boolean isShooting = false;
    private ElapsedTime pathTimer = new ElapsedTime();
    private Statemachine<AutoState> fsm = new Statemachine<AutoState>(AutoState.SHOOT_FIRST);
    private Outtake outtake;
    private IntakeIndex intakeIndex;
    public void init (){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(88.48, 134.7, Math.toRadians(90)));
        paths = new Paths(follower);
        outtake = new Outtake(hardwareMap);
        intakeIndex = new IntakeIndex(hardwareMap);
        setUp();
    }
    public void start(){
        fsm.init();
    }
    public void stop(){
        String xPose, yPose, heading;
        Pose pose = follower.getPose();
        xPose = Double.toString(pose.getX());
        yPose = Double.toString(pose.getY());
        heading = Double.toString(Math.toDegrees(pose.getHeading()));

        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        ReadWriteFile.writeFile(file, 0 + "\n" + xPose + "\n" + yPose + "\n" + heading);
    }
    public void loop(){
        follower.update();
        fsm.update();
    }
    private AutoState handleShoot(AutoState nextState, long durationMs) {
        if (!isShooting) {
            pathTimer.reset();
            isShooting = true;
            outtake.autoUpdate(true);
        }
        if (pathTimer.milliseconds() > durationMs) {
            isShooting = false;
            outtake.autoUpdate(false);
            return nextState;
        }
        return null;
    }
    private void setUp(){
        fsm.onStateEnter(AutoState.SHOOT_FIRST, () -> {
            follower.followPath(paths.SHOOT_FIRST);
            outtake.autoUpdate(false);
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_FIRST, () -> {

            if(!follower.isBusy()){
                return handleShoot(AutoState.TAKE_BALLS, 1200);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.TAKE_BALLS, () -> {
            follower.followPath(paths.TAKE_BALLS);
            outtake.autoUpdate(false);
            return null;
        });
        fsm.onStateUpdate(AutoState.TAKE_BALLS, () -> {
            intakeIndex.auto();
            if(!follower.isBusy()){
                return AutoState.GO_SHOOT;
            }
            return null;
        });
        fsm.onStateEnter(AutoState.GO_SHOOT, () -> {
            follower.followPath(paths.GO_SHOOT);
            outtake.autoUpdate(false);
            return null;
        });
        fsm.onStateUpdate(AutoState.GO_SHOOT, () -> {
            if(!follower.isBusy()){
                return handleShoot(AutoState.PARK, 1200);
            }
            return null;
        });
        fsm.onStateEnter(AutoState.PARK, () -> {
            follower.followPath(paths.PARK);
            outtake.autoUpdate(false);
            return null;
        });
        fsm.onStateUpdate(AutoState.PARK, () -> {
            if(!follower.isBusy()){
                requestOpModeStop();
            }
            return null;
        });
    }
    public static class Paths {
        public PathChain SHOOT_FIRST;
        public PathChain TAKE_BALLS;
        public PathChain GO_SHOOT;
        public PathChain PARK;
        public Paths(Follower follower) {
            SHOOT_FIRST = (follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.48, 134.7),
                                    new Pose(86, 109)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build()
            );
            TAKE_BALLS = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86, 109),
                                    new Pose(97.12162162162162, 131.75),
                                    new Pose(132, 135.6)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build()
            );
            GO_SHOOT = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(132, 135.6),
                                    new Pose(97.12162162162162, 131.75),
                                    new Pose(86, 109)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build()
            );
            PARK = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(86, 109),
                                    new Pose(123.8429054054054, 115.2),
                                    new Pose(129, 30)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build()
            );

        }
    }
}
