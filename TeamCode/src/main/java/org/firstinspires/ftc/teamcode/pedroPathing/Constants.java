package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.75)
            //.forwardZeroPowerAcceleration(-25.9346931313679598) // OLD
            .forwardZeroPowerAcceleration(-34.918377572914916) // NEW
            //.lateralZeroPowerAcceleration(-67.342491844080064) // OLD
            .lateralZeroPowerAcceleration(-75.76916150636166) // NEW

            // Primary XY position control
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.08, 0.0, 0.0, 0.01
            ))
            // Secondary translational PIDF (for finer correction)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.12, 0.0, 0.013, 0.01
            ))
            // Primary heading control (rotation)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.0, 0.0, 0.05, 0.025
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0.9, 0.0, 0.06, 0.035
            ))
            // Drive PIDF (helps keep velocity smooth)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.2, 0.0, 0.009, 0.0, 0.01
            ))
            // Secondary drive PIDF
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.5, 0.0, 0.12, 0.0, 0.01
            ))
            .drivePIDFSwitch(18)
            .translationalPIDFSwitch(15)
            .headingPIDFSwitch(Math.toRadians(15))


            // Enable the secondary loops (these help smooth out motion)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true);


    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            0.86,
            10,
            0.2
    );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.8)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            //.xVelocity(78.261926752421046666666666666667) // OLD
            .xVelocity(49.852386234313485) // NEW
            //.yVelocity(61.494551922189565); // OLD
            .yVelocity(43.82936973271408); // NEW


    /*public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .robotLength(16.5)
            .robotWidth(17)
            .forwardTicksToInches(0.006313782143400114)
            .strafeTicksToInches(0.006325054641343225)
            .turnTicksToInches(-0.11523000868339908)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD);*/


    static SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-2.875, 1.5, 0);
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            //.offset(offset)
            .linearScalar(0.9571582267593028)
            .angularScalar(0.9962755890624266);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .OTOSLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
