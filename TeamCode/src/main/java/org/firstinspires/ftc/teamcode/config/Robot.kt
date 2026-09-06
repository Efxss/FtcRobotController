package org.firstinspires.ftc.teamcode.config

import android.util.Size
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class Robot(
    hardwareMap: HardwareMap,
) {
    enum class Alliance {BLUE, RED}
    // Hardware
    // val intakeMotor: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "intake")
    val intakeMotor: MotorEx = MotorEx(hardwareMap, "intake", Motor.GoBILDA.BARE).setCachingTolerance(0.2)
    val colorSen: SensorRevColorV3 = SensorRevColorV3(hardwareMap, "c")
    val ll: Limelight3A = hardwareMap.get(Limelight3A::class.java, "LL")
    // Vision stuff
    var aprilTag: AprilTagProcessor? = null
    var visionPortal: VisionPortal? = null
    val cameraPosition: Position = Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0)
    val cameraOrientation = YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -90.0, 0.0, 0)
    var cameraRes = Size(1280, 800)
    fun initAprilTag(hardwareMap: HardwareMap, webcamName: String) {
        aprilTag = AprilTagProcessor.Builder()
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
            .setCameraPose(cameraPosition, cameraOrientation)
            .build()
        val builder = VisionPortal.Builder()
        builder.setCamera(hardwareMap.get(WebcamName::class.java, webcamName))
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        builder.setCameraResolution(cameraRes)
        builder.addProcessor(aprilTag)
        visionPortal = builder.build()
    }
    // Objects and needed classes
    object AutoPoseUtil {
        lateinit var follower: Follower
        val startPose = Pose(31.7, 8.0, Math.toRadians(180.0))
        val bottomLeftCorner = Pose(10.0, 8.0, Math.toRadians(180.0))
        val bottomRightCorner = bottomLeftCorner.mirror()!!
        val topLeftCorner = Pose(10.0, 132.0, Math.toRadians(180.0))
        val topRightCorner = topLeftCorner.mirror()!!
        val leftSpike = Pose(23.7, 85.0, Math.toRadians(90.0))
        val rightSpike = leftSpike.mirror()!!
        val hiveFourLeftSide = Pose(56.0, 132.0, Math.toRadians(0.0))
        val startToLeftCorner: PathChain by lazy { follower.pathBuilder()
            .addPath((BezierLine(startPose, bottomLeftCorner)))
            .setConstantHeadingInterpolation(bottomLeftCorner.heading)
            .build() }
        val bottomLeftCornerToLeftSpike: PathChain by lazy { follower.pathBuilder()
            .addPath((BezierLine(bottomLeftCorner, leftSpike)))
            //.setLinearHeadingInterpolation(leftSpike.heading, leftSpike.heading)
            .setHeadingInterpolation ( HeadingInterpolator.piecewise(
                HeadingInterpolator.PiecewiseNode(
                    0.0,
                    0.1,
                    HeadingInterpolator.constant(bottomLeftCorner.heading)
                ),
                HeadingInterpolator.PiecewiseNode(
                    0.1,
                    1.0,
                    HeadingInterpolator.constant(leftSpike.heading)
                )
            ) )
            .build()}
        val leftSpikeToHiveFour: PathChain by lazy { follower.pathBuilder()
            .addPath((BezierLine(leftSpike, hiveFourLeftSide)))
            .setConstantHeadingInterpolation(hiveFourLeftSide.heading)
            .build() }
        // Example to go off of
        /*val startPoseBlueDepoPose = Pose(32.7, 135.3, Math.toRadians(90.0))
        val startPoseRedDepoPose = startPoseBlueDepoPose.mirror()!!
        val BlueDepoStartScore: PathChain by lazy { follower.pathBuilder()
            .addPath((BezierLine(startPoseBlueDepoPose, startPoseRedDepoPose)))
            .setLinearHeadingInterpolation(startPoseBlueDepoPose.heading, startPoseRedDepoPose.heading)
            .build() }
        val BlueDepoMiddleSpikeGrabCurve: PathChain by lazy { follower.pathBuilder()
            .addPath(BezierCurve(BlueDepoScorePose, BlueDepoMiddleSpikeAlignmentPose, BlueDepoMiddleSpikeGrabPose))
            .setConstantHeadingInterpolation(BlueDepoMiddleSpikeGrabPose.heading)
            .build() }*/
    }
}