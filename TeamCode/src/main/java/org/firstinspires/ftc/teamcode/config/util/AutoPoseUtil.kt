package org.firstinspires.ftc.teamcode.config.util
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.pedropathing.paths.PathChain

object AutoPoseUtil {
    lateinit var follower: Follower
    val startPoseBlueDepoPose = Pose(18.1, 122.6, Math.toRadians(140.0))
    val BlueDepoEndPose = Pose(17.925, 122.425, Math.toRadians(140.0))
    val BlueDepoScorePose = Pose(65.6, 77.0, Math.toRadians(140.0))
    val BlueDepoCloseSpikeStripPose = Pose(26.0, 82.8, Math.toRadians(-180.0))
    val BlueDepoMiddleSpikeAlignmentPose = Pose(47.2, 59.0, Math.toRadians(-180.0))
    val BlueDepoMiddleSpikeGrabPose = Pose(26.0, 59.0, Math.toRadians(-180.0))
    val BlueDepoFarSpikeAlignmentPose = Pose(47.1, 35.6, Math.toRadians(-180.0))
    val BlueDepoFarSpikeGrabPose = Pose(26.0, 35.6, Math.toRadians(-180.0))
    val BlueSideSquarePose = Pose(38.7, 31.2, Math.toRadians(-180.0))
    val startPoseRedDepoPose = startPoseBlueDepoPose.mirror()!!
    val RedDepoEndPose = BlueDepoEndPose.mirror()!!
    val RedDepoScorePose = BlueDepoScorePose.withX(BlueDepoScorePose.x - 8.0).mirror()!!
    val RedDepoCloseSpikeStripPose = BlueDepoCloseSpikeStripPose.withX(BlueDepoCloseSpikeStripPose.x - 8.0).mirror()!!
    val RedDepoMiddleSpikeAlignmentPose = BlueDepoMiddleSpikeAlignmentPose.withX(BlueDepoMiddleSpikeAlignmentPose.x - 8.0).mirror()!!
    val RedDepoMiddleSpikeGrabPose = BlueDepoMiddleSpikeGrabPose.withX(BlueDepoMiddleSpikeGrabPose.x - 8.0).mirror()!!
    val RedDepoFarSpikeAlignmentPose = BlueDepoFarSpikeAlignmentPose.withX(BlueDepoFarSpikeAlignmentPose.x - 8.0).mirror()!!
    val RedDepoFarSpikeGrabPose = BlueDepoFarSpikeGrabPose.withX(BlueDepoFarSpikeGrabPose.x - 8.0).mirror()!!
    val RedSideSquarePose = BlueSideSquarePose.withX(BlueSideSquarePose.x - 8.0).mirror()!!
    val BlueDepoStartScore : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(startPoseBlueDepoPose, BlueDepoScorePose)))
        .setLinearHeadingInterpolation(startPoseBlueDepoPose.heading, BlueDepoScorePose.heading)
        .build() }
    val BlueDepoScoreToBlueDepoEnd : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(BlueDepoScorePose, BlueDepoEndPose)))
        .setLinearHeadingInterpolation(BlueDepoScorePose.heading, BlueDepoEndPose.heading)
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
    val BlueDepoScoreToRedSideSquare : PathChain by lazy { follower.pathBuilder()
        .addPath(BezierLine(BlueDepoScorePose, RedSideSquarePose))
        .setLinearHeadingInterpolation(BlueDepoScorePose.heading, RedSideSquarePose.heading)
        .build() }
    val BlueDepoScoreToBlueSideSquare : PathChain by lazy { follower.pathBuilder()
        .addPath(BezierLine(BlueDepoScorePose, BlueSideSquarePose))
        .setLinearHeadingInterpolation(BlueDepoScorePose.heading, BlueSideSquarePose.heading)
        .build() }
    val BlueSideSquareToBlueDepoScorePose : PathChain by lazy { follower.pathBuilder()
        .addPath(BezierLine(BlueSideSquarePose, BlueDepoScorePose))
        .setLinearHeadingInterpolation(BlueSideSquarePose.heading, BlueDepoScorePose.heading)
        .build() }

    val RedDepoStartScore : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(startPoseRedDepoPose, RedDepoScorePose)))
        .setLinearHeadingInterpolation(startPoseRedDepoPose.heading, RedDepoScorePose.heading)
        .build() }
    val RedDepoScoreToRedDepoEnd : PathChain by lazy { follower.pathBuilder()
        .addPath((BezierLine(RedDepoScorePose, RedDepoEndPose)))
        .setLinearHeadingInterpolation(RedDepoScorePose.heading, RedDepoEndPose.heading)
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
    val RedDepoScoreToBlueSideSquare : PathChain by lazy { follower.pathBuilder()
        .addPath(BezierLine(RedDepoScorePose, BlueSideSquarePose))
        .setLinearHeadingInterpolation(RedDepoScorePose.heading, BlueSideSquarePose.heading)
        .build() }
    val RedDepoScoreToRedSideSquare : PathChain by lazy { follower.pathBuilder()
        .addPath(BezierLine(RedDepoScorePose, RedSideSquarePose))
        .setLinearHeadingInterpolation(RedDepoScorePose.heading, RedSideSquarePose.heading)
        .build() }
    val RedSideSquareToRedDepoScorePose : PathChain by lazy { follower.pathBuilder()
        .addPath(BezierLine(RedSideSquarePose, RedDepoScorePose))
        .setLinearHeadingInterpolation(RedSideSquarePose.heading, RedDepoScorePose.heading)
        .build() }
    val RedSideSquareToBlueDepoScorePose : PathChain by lazy { follower.pathBuilder()
        .addPath(BezierLine(RedSideSquarePose, BlueDepoScorePose))
        .setLinearHeadingInterpolation(RedSideSquarePose.heading, BlueDepoScorePose.heading)
        .build() }

    fun allSpikeAutoBlue() : Command {
        return sequential(
            follow(follower, BlueDepoStartScore, true),
            follow(follower, BlueDepoCloseSpike, true),
            follow(follower, BlueDepoCloseSpikeScore, true),
            follow(follower, BlueDepoMiddleSpikeAlignment, true),
            follow(follower, BlueDepoMiddleSpikeGrab, true),
            follow(follower, BlueDepoMiddleSpikeScore, true),
            follow(follower, BlueDepoFarSpikeAlignment, true),
            follow(follower, BlueDepoFarSpikeGrab, true),
            follow(follower, BlueDepoFarSpikeScore, true),
            follow(follower, BlueDepoScoreToBlueSideSquare, true),
            waitMs(1000.0),
            follow(follower, BlueSideSquareToBlueDepoScorePose, true),
            follow(follower, BlueDepoScoreToBlueDepoEnd, true, 0.3)
        )
    }

    fun allSpikeAutoRed() : Command {
        return sequential(
            follow(follower, RedDepoStartScore, true),
            follow(follower, RedDepoCloseSpike, true),
            follow(follower, RedDepoCloseSpikeScore, true),
            follow(follower, RedDepoMiddleSpikeAlignment, true),
            follow(follower, RedDepoMiddleSpikeGrab, true),
            follow(follower, RedDepoMiddleSpikeScore, true),
            follow(follower, RedDepoFarSpikeAlignment, true),
            follow(follower, RedDepoFarSpikeGrab, true),
            follow(follower, RedDepoFarSpikeScore, true),
            follow(follower, RedDepoScoreToRedSideSquare, true),
            waitMs(1000.0),
            follow(follower, RedSideSquareToRedDepoScorePose, true),
            follow(follower, RedDepoScoreToRedDepoEnd, true, 0.3)
        )
    }
}