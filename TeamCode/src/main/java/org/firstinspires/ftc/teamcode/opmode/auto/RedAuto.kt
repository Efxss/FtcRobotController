package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.Scheduler.schedule
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.AutoPoseUtil

class RedAuto : AutoOpMode() {
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
            follow(follower, AutoPoseUtil.RedDepoStartScore, false),
            follow(follower, AutoPoseUtil.RedDepoCloseSpike, false),
            follow(follower, AutoPoseUtil.RedDepoCloseSpikeScore, false),
            follow(follower, AutoPoseUtil.RedDepoMiddleSpikeAlignment, false),
            follow(follower, AutoPoseUtil.RedDepoMiddleSpikeGrab, false),
            follow(follower, AutoPoseUtil.RedDepoMiddleSpikeScore, false),
            follow(follower, AutoPoseUtil.RedDepoFarSpikeAlignment, false),
            follow(follower, AutoPoseUtil.RedDepoFarSpikeGrab, false),
            follow(follower, AutoPoseUtil.RedDepoFarSpikeScore, false)
        )
    }
    
    fun initializePedroPathing() {
        Scheduler.reset()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(AutoPoseUtil.startPoseRedDepoPose)
        AutoPoseUtil.follower = follower
    }
}