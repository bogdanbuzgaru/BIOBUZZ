package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ServoTest")
public class ServoTest extends OpMode {
    private Servo slides;
    private int position = 0;

    public void init (){
        slides = hardwareMap.get(Servo.class, "slides");
    }
    public void loop(){
        if(gamepad1.dpadUpWasPressed() && position <= 0.9){
            position += 0.1;
        }else if (gamepad1.dpadDownWasPressed() && position >= 0.1){
            position -= 0.1;
        }else if (gamepad1.dpadLeftWasPressed() && position >= 0.05){
            position -= 0.05;
        }else if (gamepad1.dpadRightWasPressed() && position <= 0.95){
            position += 0.05;
        }
        slides.setPosition(position);
        telemetry.addData("Position", position);
    }
}
