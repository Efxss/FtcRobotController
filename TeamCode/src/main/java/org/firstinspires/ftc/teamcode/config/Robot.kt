package org.firstinspires.ftc.teamcode.config

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import com.seattlesolvers.solverslib.util.InterpLUT
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

class Robot(
    hardwareMap: HardwareMap,
) {
    enum class Alliance {BLUE, RED}
    enum class OpMode {AUTO, TELEOP}
    // PedroPathing
    lateinit var follower: Follower
    fun initPedro(hardwareMap: HardwareMap, opmode: Robot.OpMode) {
        when (opmode) {
            OpMode.AUTO -> {
                follower = Constants.createFollower(hardwareMap)
                follower.setStartingPose(Robot.AutoPoseUtil.startPose)
                Robot.AutoPoseUtil.follower = follower
            }
            OpMode.TELEOP -> {
                val startPose = VariableStateUtil.endOfAutoPose ?: Pose()
                follower = Constants.createFollower(hardwareMap)
                follower.setStartingPose(startPose)
            }
        }
    }
    // Hardware
    val intakeM: MotorEx = MotorEx(hardwareMap, "intake", 28.0, 6000.0).setCachingTolerance(0.2)
    val fireM: MotorEx = MotorEx(hardwareMap, "fire", 28.0, 6000.0).setCachingTolerance(0.05)
    // Tables and refs
    val refPose = Pose(0.0,0.0)
    val upFireBlueTab = InterpLUT()
    val downFireBlueTab = InterpLUT()
    val upFireRedTab = InterpLUT()
    val downFireRedTab = InterpLUT()
    val upFireHeadBlueTab = InterpLUT()
    val downFireHeadBlueTab = InterpLUT()
    val upFireHeadRedTab = InterpLUT()
    val downFireHeadRedTab = InterpLUT()
    // Objects, Classes and Functions
    fun genTab() {
        // Fire Power
        upFireBlueTab.apply {
            add(Pose().distanceFrom(refPose), 0.1)
        }.createLUT()

        downFireBlueTab.apply {
            add(Pose().distanceFrom(refPose), 0.1)
        }.createLUT()

        upFireRedTab.apply {
            add(Pose().distanceFrom(refPose), 0.1)
        }.createLUT()

        downFireRedTab.apply {
            add(Pose().distanceFrom(refPose), 0.1)
        }.createLUT()

        // Turn Heading
        upFireHeadBlueTab.apply {
            add(Pose(85.8, 133.2).distanceFrom(refPose), 270.0)
        }.createLUT()

        downFireHeadBlueTab.apply {
            add(Pose().distanceFrom(refPose), 0.1)
        }.createLUT()

        upFireHeadRedTab.apply {
            add(Pose().distanceFrom(refPose), 0.1)
        }.createLUT()

        downFireHeadRedTab.apply {
            add(Pose(16.4,12.0).distanceFrom(refPose), 44.0)
            add(Pose(33.7,12.0).distanceFrom(refPose), 56.0)
            add(Pose(62.2,8.0).distanceFrom(refPose), 90.0)
            add(Pose(80.0,12.0).distanceFrom(refPose), 124.0)
            add(Pose(96.5,17.5).distanceFrom(refPose), 137.0)
            add(Pose(110.4,12.0).distanceFrom(refPose), 141.0)
        }.createLUT()
    }
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