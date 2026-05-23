package org.firstinspires.ftc.teamcode.config.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.045841157247118285, 0.0017797337397567827))
            .centripetalScaling(0)
            .turnHeadingErrorThreshold(2.0)
            .mass(6.7);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.95,
            100.0,
            0.2,
            0.009,
            10.0,
            0.25,
            10,
            1.0
    );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .useBrakeModeInTeleOp(true)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
            //.xVelocity(67.80789593073327)
            //.yVelocity(63.19661778727855);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.396582896315202)
            .strafePodX(0.34288685716042977)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pnpt")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}