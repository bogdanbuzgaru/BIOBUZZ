package org.firstinspires.ftc.teamcode.pedropathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(15.2)
//            .forwardZeroPowerAcceleration(-35.69158106215242)
//            .lateralZeroPowerAcceleration(-60.6091456069354);

//            // HEADING PID
//            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.09, 0.04))
//            .useSecondaryHeadingPIDF(false)
//
//            // TRANSLATIONAL PID
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0.03))
//            .useSecondaryTranslationalPIDF(false)
//
//            // DRIVE PID
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0, 0.0008, 0.6, 0.01))
//            .useSecondaryDrivePIDF(false)
//
//            //CENTRIPEDAL
//            .centripetalScaling(0);
//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(15.2)
//            .forwardZeroPowerAcceleration(-36.60550623816884)
//            .lateralZeroPowerAcceleration(-60.7838173444961)
//
//            // Heading PID
//            .headingPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.004, 0.1))
//            .useSecondaryHeadingPIDF(true)
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.004, 0.06))
//
//            // Translational PID
//            .translationalPIDFCoefficients(new PIDFCoefficients(1, 0.005, 0.04, 0.1))
//            .useSecondaryTranslationalPIDF(true)
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.4, 0.0006, 0.01, 0.06))
//
//            // Drive PID
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.8, 0.00005, 0.02, 0, 0.05))
//            .useSecondaryDrivePIDF(true)
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.0018, 0.006, 0.0008, 0, 0.03))
//
//            .centripetalScaling(0.005);
//            .translationalPIDFCoefficients(new PIDFCoefficients(1, 0.0005, 0.04, 0.1))
//            .useSecondaryTranslationalPIDF(true)
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.4,0.0006,0.01,0.06))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.004, 0.06))
//            .useSecondaryHeadingPIDF(true)
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.55, 0.006, 0.04, 0.06));
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.8,0.00005,0.02,0,0.05))
//            .useSecondaryDrivePIDF(true)
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.0018, 0.006,0.0008, 0, 0.03))
//            .centripetalScaling(0.005)
            ;


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.9, 0.25);

    public static PinpointConstants localizerConstants = new PinpointConstants()
//            .forwardPodY(92.5)
//            .strafePodX(-141)
//            .distanceUnit(DistanceUnit.MM)
//            .strafePodX(-3.5433070866)
//            .forwardPodY(4.2519685039)
//            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .xVelocity(71.1380620040293)
//            .yVelocity(55.06179401067298)
            ;
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

}