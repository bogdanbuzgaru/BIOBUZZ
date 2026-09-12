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
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setTargetPosition(0);
        slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public void loop(){
        if(gamepad1.dpadUpWasPressed()){
            position += 100;
        }else if (gamepad1.dpadDownWasPressed()){
            position -= 100;
        }else if (gamepad1.dpadLeftWasPressed()){
            position -= 10;
        }else if (gamepad1.dpadRightWasPressed()){
            position += 10;
        }
        slides.setTargetPosition(position);
        slides.setPower(1);
        telemetry.addData("Position", position);
    }
}
