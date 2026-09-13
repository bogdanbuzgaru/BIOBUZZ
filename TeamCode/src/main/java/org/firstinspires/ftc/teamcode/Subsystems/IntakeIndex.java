package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeIndex {
    DcMotor intakeMotor;
    DcMotor indexMotor;

    public IntakeIndex(HardwareMap hardwareMap)
    {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        indexMotor = hardwareMap.get(DcMotor.class, "indexMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        indexMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void auto(){
        intakeMotor.setPower(1);
        indexMotor.setPower(1);
    }
    public void activateDeactivate(float rightTrigger)
    {
        if (rightTrigger > 0.2)
        {
            intakeMotor.setPower(rightTrigger);
            indexMotor.setPower(rightTrigger);
        }
        else
        {
            intakeMotor.setPower(0);
            indexMotor.setPower(0);
        }
    }
}
