package org.firstinspires.ftc.teamcode.config.util
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain

object AutoPoseUtil {
    lateinit var follower: Follower
    val startPoseBlueDepo = Pose(18.1, 122.6, Math.toRadians(144.0))
    val BlueDepoScorePose = Pose(57.6, 83.2, Math.toRadians(144.0))
    val BlueDepoCloseSpikeStrip = Pose(18.0, 82.8, Math.toRadians(-180.0))
    val BlueDepoCloseSpikeScore = Pose(57.6, 83.2, Math.toRadians(144.0))
    var BlueDepoStartScore : PathChain = follower.pathBuilder()
        .addPath((BezierLine(startPoseBlueDepo, BlueDepoScorePose)))
        .setLinearHeadingInterpolation(startPoseBlueDepo.heading, BlueDepoScorePose.heading)
        .build()
    var BlueDepoSpikeClose : PathChain = follower.pathBuilder()
        .addPath((BezierLine(BlueDepoScorePose, BlueDepoCloseSpikeStrip)))
        .setLinearHeadingInterpolation(BlueDepoScorePose.heading, BlueDepoCloseSpikeStrip.heading)
        .build()
    var BlueDepoSpikeCloseScore : PathChain = follower.pathBuilder()
        .addPath((BezierLine(BlueDepoCloseSpikeStrip, BlueDepoCloseSpikeScore)))
        .setLinearHeadingInterpolation(BlueDepoCloseSpikeStrip.heading, BlueDepoCloseSpikeScore.heading)
        .build()
}