package org.firstinspires.ftc.teamcode.opmode.auto.red

import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.Alliance
import org.firstinspires.ftc.teamcode.config.util.AutoPoseUtil

@Autonomous(group = "Red Auto", name = "Auto Pod Path Test Red")
class RedAuto : AutoOpMode() {
    lateinit var follower: Follower
    override val alliance = Alliance.RED
    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        Scheduler.schedule(runAuto())
    }

    override fun onLoop() {
        follower.update()
        Scheduler.execute()
    }

    fun runAuto() : Command {
        return Groups.sequential(
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.RedDepoStartScore, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.RedDepoCloseSpike, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.RedDepoCloseSpikeScore, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.RedDepoMiddleSpikeAlignment, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.RedDepoMiddleSpikeGrab, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.RedDepoMiddleSpikeScore, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.RedDepoFarSpikeAlignment, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.RedDepoFarSpikeGrab, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.RedDepoFarSpikeScore, false)
        )
    }

    fun initializePedroPathing() {
        Scheduler.reset()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(AutoPoseUtil.startPoseRedDepoPose)
        AutoPoseUtil.follower = follower
    }
}