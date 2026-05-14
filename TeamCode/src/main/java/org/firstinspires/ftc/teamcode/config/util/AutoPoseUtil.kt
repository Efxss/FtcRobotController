package org.firstinspires.ftc.teamcode.config.util
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.pedropathing.paths.PathChain

object AutoPoseUtil {
    lateinit var follower: Follower
    val startPoseBlueDepoPose = Pose(18.1, 122.6, Math.toRadians(140.0))
    val BlueDepoScorePose = Pose(65.6, 77.0, Math.toRadians(140.0))
    val BlueDepoCloseSpikeStripPose = Pose(26.0, 82.8, Math.toRadians(-180.0))
    val BlueDepoMiddleSpikeAlignmentPose = Pose(47.2, 59.0, Math.toRadians(-180.0))
    val BlueDepoMiddleSpikeGrabPose = Pose(26.0, 59.0, Math.toRadians(-180.0))
    val BlueDepoFarSpikeAlignmentPose = Pose(47.1, 35.6, Math.toRadians(-180.0))
    val BlueDepoFarSpikeGrabPose = Pose(26.0, 35.6, Math.toRadians(-180.0))
    val startPoseRedDepoPose = startPoseBlueDepoPose.mirror()!!
    val RedDepoScorePose = BlueDepoScorePose.withX(BlueDepoScorePose.x - 8.0).mirror()!!
    val RedDepoCloseSpikeStripPose = BlueDepoCloseSpikeStripPose.withX(BlueDepoCloseSpikeStripPose.x - 8.0).mirror()!!
    val RedDepoMiddleSpikeAlignmentPose = BlueDepoMiddleSpikeAlignmentPose.withX(BlueDepoMiddleSpikeAlignmentPose.x - 8.0).mirror()!!
    val RedDepoMiddleSpikeGrabPose = BlueDepoMiddleSpikeGrabPose.withX(BlueDepoMiddleSpikeGrabPose.x - 8.0).mirror()!!
    val RedDepoFarSpikeAlignmentPose = BlueDepoFarSpikeAlignmentPose.withX(BlueDepoFarSpikeAlignmentPose.x - 8.0).mirror()!!
    val RedDepoFarSpikeGrabPose = BlueDepoFarSpikeGrabPose.withX(BlueDepoFarSpikeGrabPose.x - 8.0).mirror()!!
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

    fun allSpikeAutoBlue() : Command {
        return sequential(
            follow(follower, BlueDepoStartScore, false),
            follow(follower, BlueDepoCloseSpike, false),
            follow(follower, BlueDepoCloseSpikeScore, false),
            follow(follower, BlueDepoMiddleSpikeAlignment, false),
            follow(follower, BlueDepoMiddleSpikeGrab, false),
            follow(follower, BlueDepoMiddleSpikeScore, false),
            follow(follower, BlueDepoFarSpikeAlignment, false),
            follow(follower, BlueDepoFarSpikeGrab, false),
            follow(follower, BlueDepoFarSpikeScore, false)
        )
    }

    fun allSpikeAutoRed() : Command {
        return sequential(
            follow(follower, RedDepoStartScore, false),
            follow(follower, RedDepoCloseSpike, false),
            follow(follower, RedDepoCloseSpikeScore, false),
            follow(follower, RedDepoMiddleSpikeAlignment, false),
            follow(follower, RedDepoMiddleSpikeGrab, false),
            follow(follower, RedDepoMiddleSpikeScore, false),
            follow(follower, RedDepoFarSpikeAlignment, false),
            follow(follower, RedDepoFarSpikeGrab, false),
            follow(follower, RedDepoFarSpikeScore, false)
        )
    }
}