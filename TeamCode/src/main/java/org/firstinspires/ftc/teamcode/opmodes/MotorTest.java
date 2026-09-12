package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="MotorTest")
public class MotorTest extends OpMode {
    private DcMotorEx slides;
    private int position = 0;

    public void init (){
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        slides.setTargetPosition(0);
        slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public void loop(){
        if(gamepad1.dpadUpWasPressed()){
            position += 100;
        }else if (gamepad1.dpadDownWasPressed()){
            position -= 100;
        }
        slides.setTargetPosition(position);
        telemetry.addData("Position", position);
    }
}
