package org.firstinspires.ftc.teamcode.config.util
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain

object AutoPoseUtil {
    lateinit var follower: Follower
    val startPose = Pose(31.7, 8.0, Math.toRadians(180.0))
    val startCorner = Pose(12.0, 9.0, Math.toRadians(180.0))
    val leftSpike = Pose(23.7, 75.2, Math.toRadians(90.0))
    val hiveFourLeftSide = Pose(54.0, 132.0, Math.toRadians(0.0))
    val startToCornerToSpike: PathChain by lazy { follower.pathBuilder()
        .addPath(BezierCurve(startPose, startCorner, leftSpike))
        .setLinearHeadingInterpolation(startPose.heading, startCorner.heading, leftSpike.heading)
        .build() }
    val leftSpikeToHiveFour: PathChain by lazy { follower.pathBuilder()
        .addPath(BezierLine(leftSpike, hiveFourLeftSide))
        .setConstantHeadingInterpolation(0.0)
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