package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp
public class TuneFlyWheel extends OpMode {
    private DcMotorEx leftShooter;
    private double lowVelocity = 2500;
    private double midVelocity = 1100;
    private double highVelocity = 1700;
    private double veryhighVelocity = 2100;
    private double currentVelocity = 0;

    public static double P = 29.6;//21.5
    public static double F = 0.3;

    private double tuneSteps[] = {10.00, 1.00, 0.1, 0.01 };
    private double stepsize = 0, error = 0;
    private int counter = 0, velCounter = 0;
    public static ElapsedTime time = new ElapsedTime();
    public static double vel;

//    public static val panelsTelemetry = Panels.;

    public void init(){
        leftShooter = hardwareMap.get(DcMotorEx.class, "flywheel1");
//        rightShooter = hardwareMap.get(DcMotorEx.class, "flywheel2");
        leftShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftShooter.setDirection(DcMotorEx.Direction.FORWARD);
//        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addLine("Initialized");
    }
    public void start(){
        time.reset();
    }
    public void loop(){
        tune(gamepad1);
        vel = leftShooter.getVelocity();
    }
    private void tune(Gamepad gamepad) {

        if(gamepad.squareWasPressed()){
            switch (velCounter){
                case 0 -> velCounter = 1;
                case 1 -> velCounter = 2;
                case 2 -> velCounter = 3;
                case 3 -> velCounter = 0;
            }
        }else if(gamepad.circleWasPressed()) {
            switch (velCounter) {
                case 0 -> velCounter = 3;
                case 1 -> velCounter = 0;
                case 2 -> velCounter = 1;
                case 3 -> velCounter = 2;
            }
        }

        switch (velCounter){
            case 0 -> currentVelocity = lowVelocity;
            case 1 -> currentVelocity = midVelocity;
            case 2 -> currentVelocity = highVelocity;
            case 3 -> currentVelocity = veryhighVelocity;
        }
        leftShooter.setVelocity(currentVelocity);
//        rightShooter.setVelocity(currentVelocity);

        if(gamepad.rightBumperWasPressed()){
            switch (counter){
                case 0 -> counter = 1;
                case 1 -> counter = 2;
                case 2 -> counter = 3;
                case 3 -> counter = 0;
            }
            stepsize = tuneSteps[counter];
        }else if (gamepad.leftBumperWasPressed()){
            switch (counter){
                case 0 -> counter = 3;
                case 1 -> counter = 0;
                case 2 -> counter = 1;
                case 3 -> counter = 2;
            }
            stepsize = tuneSteps[counter];
        }

        if(gamepad.dpadUpWasPressed()){
            P += stepsize;
        }else if(gamepad.dpadDownWasPressed()){
            P -= stepsize;
        }


        if(gamepad.dpadLeftWasPressed()){
            F -= stepsize;
        }else if(gamepad.dpadRightWasPressed()){
            F += stepsize;
        }


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        leftShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//        rightShooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        error = currentVelocity - leftShooter.getVelocity();
        if (error < 0) {
            telemetry.addLine("ERROR is NEGATIVE");
        } else {
            telemetry.addLine("ERROR is POSITIVE");
        }

        telemetry.addData("time", time);
        telemetry.addData("velocity", vel);
        telemetry.addData("target", currentVelocity);
        telemetry.addLine("---------------------------------------------");
        telemetry.addLine();
        telemetry.addData("P", P);
        telemetry.addData("F", F);
        telemetry.addData("Step", stepsize);
        telemetry.update();
    }

}
