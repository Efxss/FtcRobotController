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
    val startPoseRedDepoPose = Pose(18.1, 122.6, Math.toRadians(144.0)).mirror()!!
    val RedDepoScorePose = Pose(57.6, 83.2, Math.toRadians(144.0)).mirror()!!
    val RedDepoCloseSpikeStripPose = Pose(18.0, 82.8, Math.toRadians(-180.0)).mirror()!!
    val RedDepoMiddleSpikeAlignmentPose = Pose(39.2, 59.0, Math.toRadians(-180.0)).mirror()!!
    val RedDepoMiddleSpikeGrabPose = Pose(18.0, 59.0, Math.toRadians(-180.0)).mirror()!!
    val RedDepoFarSpikeAlignmentPose = Pose(39.1, 35.6, Math.toRadians(-180.0)).mirror()!!
    val RedDepoFarSpikeGrabPose = Pose(18.0, 35.6, Math.toRadians(-180.0)).mirror()!!
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

    val RedDepoStartScore : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(startPoseRedDepoPose, RedDepoScorePose)))
        .setLinearHeadingInterpolation(startPoseRedDepoPose.heading, RedDepoScorePose.heading)
        .build() }
    val RedDepoCloseSpike : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(RedDepoScorePose, RedDepoCloseSpikeStripPose)))
        .setLinearHeadingInterpolation(RedDepoScorePose.heading, RedDepoCloseSpikeStripPose.heading)
        .build() }
    val RedDepoCloseSpikeScore : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(RedDepoCloseSpikeStripPose, RedDepoScorePose)))
        .setLinearHeadingInterpolation(RedDepoCloseSpikeStripPose.heading, RedDepoScorePose.heading)
        .build() }
    val RedDepoMiddleSpikeAlignment : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(RedDepoScorePose, RedDepoMiddleSpikeAlignmentPose)))
        .setLinearHeadingInterpolation(RedDepoScorePose.heading, RedDepoMiddleSpikeAlignmentPose.heading)
        .build() }
    val RedDepoMiddleSpikeGrab : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(RedDepoMiddleSpikeAlignmentPose, RedDepoMiddleSpikeGrabPose)))
        .setLinearHeadingInterpolation(RedDepoMiddleSpikeAlignmentPose.heading, RedDepoMiddleSpikeGrabPose.heading)
        .build() }
    val RedDepoMiddleSpikeScore : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(RedDepoMiddleSpikeGrabPose, RedDepoScorePose)))
        .setLinearHeadingInterpolation(RedDepoMiddleSpikeGrabPose.heading, RedDepoScorePose.heading)
        .build() }
    val RedDepoFarSpikeAlignment : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(RedDepoScorePose, RedDepoFarSpikeAlignmentPose)))
        .setLinearHeadingInterpolation(RedDepoScorePose.heading, RedDepoFarSpikeAlignmentPose.heading)
        .build() }
    val RedDepoFarSpikeGrab : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(RedDepoFarSpikeAlignmentPose, RedDepoFarSpikeGrabPose)))
        .setLinearHeadingInterpolation(RedDepoFarSpikeAlignmentPose.heading, RedDepoFarSpikeGrabPose.heading)
        .build() }
    val RedDepoFarSpikeScore : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(RedDepoFarSpikeGrabPose, RedDepoScorePose)))
        .setLinearHeadingInterpolation(RedDepoFarSpikeGrabPose.heading, RedDepoScorePose.heading)
        .build() }
}