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

@Autonomous(group = "Blue Auto", name = "Auto Pod Path Test Blue")
class BlueAuto : AutoOpMode() {
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
        getDebugUtil().showAllDebug(follower, getHubUtil(), runtime)
        getDebugUtil().update(telemetry)
    }
    fun runAuto() : Command {
        return sequential(
            follow(follower, AutoPoseUtil.BlueDepoStartScore, false),
            follow(follower, AutoPoseUtil.BlueDepoCloseSpike, false),
            follow(follower, AutoPoseUtil.BlueDepoCloseSpikeScore, false),
            follow(follower, AutoPoseUtil.BlueDepoMiddleSpikeAlignment, false),
            follow(follower, AutoPoseUtil.BlueDepoMiddleSpikeGrab, false),
            follow(follower, AutoPoseUtil.BlueDepoMiddleSpikeScore, false),
            follow(follower, AutoPoseUtil.BlueDepoFarSpikeAlignment, false),
            follow(follower, AutoPoseUtil.BlueDepoFarSpikeGrab, false),
            follow(follower, AutoPoseUtil.BlueDepoFarSpikeScore, false)
        )
    }
    fun initializePedroPathing() {
        Scheduler.reset()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(AutoPoseUtil.startPoseBlueDepoPose)
        AutoPoseUtil.follower = follower
    }
}