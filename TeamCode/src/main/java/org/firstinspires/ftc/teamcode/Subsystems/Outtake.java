package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    private DcMotorEx slides;
    private DcMotorEx flywheel;
    private Servo hood, box;
    private boolean isOuttake = false;
    private int position = 550;
    private int ticksPerSec = 1300;
    public Outtake(HardwareMap hardwareMap){
        slides = hardwareMap.get(DcMotorEx.class, "slides");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        hood = hardwareMap.get(Servo.class, "hood");
        box = hardwareMap.get(Servo.class, "box");
        slides.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slides.setTargetPosition(0);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(330,0,0,12.41));
    }
    public void update(Gamepad gamepad){
        //550
        if(gamepad.crossWasPressed()){
            slides.setTargetPosition(0);
            box.setPosition(0.2);
        }else if(gamepad.triangleWasPressed()){
            slides.setTargetPosition(position);
            box.setPosition(0.8);
        }
        if(gamepad.circleWasPressed()){
            if(!isOuttake) {
                ticksPerSec = 300;
                hood.setPosition(0.9);
            }else{
                ticksPerSec = 1300;
                hood.setPosition(0.2);
            }
        }
        flywheel.setVelocity(ticksPerSec);
    }

}
