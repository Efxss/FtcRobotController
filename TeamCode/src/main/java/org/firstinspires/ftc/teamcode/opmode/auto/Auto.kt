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
        getDebugUtil().showAllDebug(follower)
    }
    fun runAuto() : Command {
        return sequential(
            follow(follower, AutoPoseUtil.BlueDepoStartScore, true),
            follow(follower, AutoPoseUtil.BlueDepoSpikeClose, true),
            follow(follower, AutoPoseUtil.BlueDepoSpikeCloseScore, true)
        )
    }
    fun initializePedroPathing() {
        Scheduler.reset()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(AutoPoseUtil.startPoseBlueDepo)
    }
}