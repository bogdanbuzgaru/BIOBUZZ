package org.firstinspires.ftc.teamcode.movement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class Movement {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftRear;
    private DcMotor rightRear;
    private int off = 0;
    public Movement(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = hardwareMap.get(DcMotor.class, "frontRight");
        leftRear = hardwareMap.get(DcMotor.class, "backLeft");
        rightRear = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left motors
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        List<DcMotor> movementMotors = Arrays.asList(leftFront, rightFront, leftRear, rightRear);
        movementMotors.forEach(motor -> motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE));


    }
    public void movementFieldCentric(Gamepad gamepad, double yaw, boolean red) {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;  //heading

        double botHeading = yaw;
        if(red){
            botHeading = botHeading;
        }else{
            botHeading += Math.PI;
        }

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double frontLeftPower = (rotY + rotX + rx);
        double backLeftPower = (rotY - rotX + rx);
        double frontRightPower = (rotY - rotX - rx);
        double backRightPower = (rotY + rotX - rx);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFront.setPower(frontLeftPower / denominator);
        leftRear.setPower(backLeftPower / denominator);
        rightFront.setPower(frontRightPower / denominator);
        rightRear.setPower(backRightPower / denominator);
    }


    public void movementLoop(Gamepad gamepad) {
        double x = gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double rx = gamepad.right_stick_x;

        x = x * 1.1;

        double frontLeftPower = (y + x + rx);
        double backLeftPower = (y - x + rx);
        double frontRightPower = (y - x - rx);
        double backRightPower = (y + x - rx);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        leftFront.setPower(frontLeftPower / denominator);
        rightFront.setPower(frontRightPower / denominator);
        leftRear.setPower(backLeftPower / denominator);
        rightRear.setPower(backRightPower / denominator);
    }

}