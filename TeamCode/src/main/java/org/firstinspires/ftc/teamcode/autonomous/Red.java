package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Subsystems.Outtake;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.statemachine.Statemachine;

import java.io.File;
import java.nio.file.Paths;

@Autonomous
public class Red extends OpMode {
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
    public void init (){
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(55.52, 9.3, Math.toRadians(-90)));
        paths = new Paths(follower);
        outtake = new Outtake(hardwareMap);
    }
    public void start(){
        setUp();
        fsm.init();
    }
    public void stop(){
        String xPose, yPose, heading;
        Pose pose = follower.getPose();
        xPose = Double.toString(pose.getX());
        yPose = Double.toString(pose.getY());
        heading = Double.toString(Math.toDegrees(pose.getHeading()));

        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        ReadWriteFile.writeFile(file, 1 + "\n" + xPose + "\n" + yPose + "\n" + heading);
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
            return null;
        });
        fsm.onStateUpdate(AutoState.SHOOT_FIRST, () -> {

            if(!follower.isBusy()){
                return AutoState.TAKE_BALLS;
            }
            return null;
        });
        fsm.onStateEnter(AutoState.TAKE_BALLS, () -> {
            follower.followPath(paths.TAKE_BALLS);
        });
        fsm.onStateUpdate(AutoState.TAKE_BALLS, () -> {

        });
        fsm.onStateEnter(AutoState.GO_SHOOT, () -> {
            follower.followPath(paths.GO_SHOOT);

        });
        fsm.onStateUpdate(AutoState.GO_SHOOT, () -> {

        });
        fsm.onStateEnter(AutoState.PARK, () -> {
            follower.followPath(paths.PARK);

        });
        fsm.onStateUpdate(AutoState.PARK, () -> {

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
                                    new Pose(55.52, 9.3),
                                    new Pose(58, 35)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-90))
                    .build()
            );
            TAKE_BALLS = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58, 35),
                                    new Pose(46.87837837837838, 12.25),
                                    new Pose(12, 9.4)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                    .build()
            );
            GO_SHOOT = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(12, 9.4),
                                    new Pose(46.87837837837838, 12.25),
                                    new Pose(58, 35)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                    .build()
            );
            PARK = (follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(58, 35),
                                    new Pose(20.15709459459459, 28.800675675675674),
                                    new Pose(16, 102)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                    .build()
            );

        }
    }
}
