package org.firstinspires.ftc.teamcode.config.util
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain

object AutoPoseUtil {
    lateinit var follower: Follower
    val startPoseBlueDepoPose = Pose(18.1, 122.6, Math.toRadians(144.0))
    val BlueDepoScorePose = Pose(57.6, 83.2, Math.toRadians(144.0))
    val BlueDepoCloseSpikeStripPose = Pose(18.0, 82.8, Math.toRadians(-180.0))
    val BlueDepoMiddleSpikeAlignmentPose = Pose(39.2, 59.0, Math.toRadians(-180.0))
    val BlueDepoMiddleSpikeGrabPose = Pose(18.0, 59.0, Math.toRadians(-180.0))
    val BlueDepoFarSpikeAlignmentPose = Pose(39.1, 35.6, Math.toRadians(-180.0))
    val BlueDepoFarSpikeGrabPose = Pose(18.0, 35.6, Math.toRadians(-180.0))
    val BlueDepoStartScore : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(startPoseBlueDepoPose, BlueDepoScorePose)))
        .setLinearHeadingInterpolation(startPoseBlueDepoPose.heading, BlueDepoScorePose.heading)
        .build() }
    val BlueDepoCloseSpike : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(BlueDepoScorePose, BlueDepoCloseSpikeStripPose)))
        .setLinearHeadingInterpolation(BlueDepoScorePose.heading, BlueDepoCloseSpikeStripPose.heading)
        .build() }
    val BlueDepoCloseSpikeScore : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(BlueDepoCloseSpikeStripPose, BlueDepoScorePose)))
        .setLinearHeadingInterpolation(BlueDepoCloseSpikeStripPose.heading, BlueDepoScorePose.heading)
        .build() }
    val BlueDepoMiddleSpikeAlignment : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(BlueDepoScorePose, BlueDepoMiddleSpikeAlignmentPose)))
        .setLinearHeadingInterpolation(BlueDepoScorePose.heading, BlueDepoMiddleSpikeAlignmentPose.heading)
        .build() }
    val BlueDepoMiddleSpikeGrab : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(BlueDepoMiddleSpikeAlignmentPose, BlueDepoMiddleSpikeGrabPose)))
        .setLinearHeadingInterpolation(BlueDepoMiddleSpikeAlignmentPose.heading, BlueDepoMiddleSpikeGrabPose.heading)
        .build() }
    val BlueDepoMiddleSpikeScore : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(BlueDepoMiddleSpikeGrabPose, BlueDepoScorePose)))
        .setLinearHeadingInterpolation(BlueDepoMiddleSpikeGrabPose.heading, BlueDepoScorePose.heading)
        .build() }
    val BlueDepoFarSpikeAlignment : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(BlueDepoScorePose, BlueDepoFarSpikeAlignmentPose)))
        .setLinearHeadingInterpolation(BlueDepoScorePose.heading, BlueDepoFarSpikeAlignmentPose.heading)
        .build() }
    val BlueDepoFarSpikeGrab : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(BlueDepoFarSpikeAlignmentPose, BlueDepoFarSpikeGrabPose)))
        .setLinearHeadingInterpolation(BlueDepoFarSpikeAlignmentPose.heading, BlueDepoFarSpikeGrabPose.heading)
        .build() }
    val BlueDepoFarSpikeScore : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(BlueDepoFarSpikeGrabPose, BlueDepoScorePose)))
        .setLinearHeadingInterpolation(BlueDepoFarSpikeGrabPose.heading, BlueDepoScorePose.heading)
        .build() }
}