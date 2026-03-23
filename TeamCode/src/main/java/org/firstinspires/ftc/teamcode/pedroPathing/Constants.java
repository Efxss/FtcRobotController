package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.67)
            .forwardZeroPowerAcceleration(-34.918377572914916)
            .lateralZeroPowerAcceleration(-75.76916150636166)

            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.12, 0.0, 0.013, 0.01
            ))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.2, 0.0, 0.0228, 0.032
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.0, 0.0, 0.05, 0.025
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    0.9, 0.0, 0.06, 0.035
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.12, 0.0, 0.013, 0.1 ,0.01
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.2, 0.0, 0.0228, 0.1 ,0.032
            ))
            .drivePIDFSwitch(8)
            .translationalPIDFSwitch(5)
            .headingPIDFSwitch(Math.toRadians(5))

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.35,
            10,
            0.2
    );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(49.852386234313485)
            .yVelocity(43.82936973271408);
    
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
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