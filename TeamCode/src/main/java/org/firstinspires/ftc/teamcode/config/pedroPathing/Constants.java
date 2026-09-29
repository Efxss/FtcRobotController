package org.firstinspires.ftc.teamcode.config.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("frontLeft");
                c.backLeftName.set("backLeft");
                c.frontRightName.set("frontRight");
                c.backRightName.set("backRight");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

                c.manualBrakeMode.set(true);
                c.powerThreshold.set(0.03);
            }
    );
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pnpt");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.4581793837659935);
        c.yPodOffset.set(2.5530450175127646);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.22863612311088866);
                Controller secondaryTranslationalForward = Controller.proportional(0.08447493821974779);
                Controller primaryTranslationalLateral = Controller.proportional(0.3039501443158016);
                Controller secondaryTranslationalLateral = Controller.proportional(0.11230145662725313);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011669649014028964));
                c.brake.set(Controller.proportionalFeedforward(0.00991920166192462));

                c.headingFeedback.set(Controller.proportional(3.9745165587114633));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05139644231664129, 0.005030310982163734));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06111462553979209, 0.07181293977779614));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015884600593932807, 0.0013628801964345244));

                c.maxAchievableForwardVelocity.set(88.72803066976005);
                c.maxAchievableStrafeVelocity.set(74.32882465907434);
                c.naturalForwardDeceleration.set(50.117862913393225);
                c.naturalStrafeDeceleration.set(75.85725310915278);
            }
    );
    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}