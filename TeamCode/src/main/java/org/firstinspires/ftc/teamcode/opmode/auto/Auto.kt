package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.Scheduler.schedule
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.AutoPoseUtil

@Autonomous(group = "Blue Auto", name = "Auto Pod Path Test")
class Auto : AutoOpMode() {
    lateinit var follower: Follower
    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        schedule(runAuto())
    }

    override fun onLoop() {
        follower.update()
        Scheduler.execute()
        getDebugUtil().showAllDebug(follower, getHubUtil())
        getDebugUtil().update(telemetry)
    }
    fun runAuto() : Command {
        return sequential(
            follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoStartScore, false),
            follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoCloseSpike, false),
            follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoCloseSpikeScore, false),
            follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoMiddleSpikeAlignment, false),
            follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoMiddleSpikeGrab, false),
            follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoMiddleSpikeScore, false)
        )
    }
    fun initializePedroPathing() {
        Scheduler.reset()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(AutoPoseUtil.startPoseBlueDepoPose)
        AutoPoseUtil.follower = follower
    }
}