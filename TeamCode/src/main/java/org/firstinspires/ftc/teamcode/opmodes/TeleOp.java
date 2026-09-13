package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.pedropathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    private final List<Double> results = new ArrayList<>();

    public void init(){
        File file = AppUtil.getInstance().getSettingsFile("FinalPos.txt");
        try {
            String[] vals = ReadWriteFile.readFile(file).split("\n");
            for (String val : vals) {
                results.add(Double.parseDouble(val));
            }
        } catch (Exception e) {
            results.add(0.0); results.add(0.0); results.add(0.0); results.add(0.0);
        }
        double startX = results.get(results.size() - 3);
        double startY = results.get(results.size() - 2);
        double startHeadingDeg = Math.toRadians(results.get(results.size() - 1));
        Pose startPose = new Pose(startX, startY, startHeadingDeg);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setPose(startPose);
    }
    public void loop (){
        follower.update();
    }
}
