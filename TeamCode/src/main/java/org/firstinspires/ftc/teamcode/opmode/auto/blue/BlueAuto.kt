package org.firstinspires.ftc.teamcode.opmode.auto.blue

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

@Autonomous(group = "Blue Auto", name = "Auto Pod Path Test Blue")
class BlueAuto : AutoOpMode() {
    lateinit var follower: Follower
    override val alliance = Alliance.BLUE
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
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoStartScore, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoCloseSpike, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoCloseSpikeScore, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoMiddleSpikeAlignment, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoMiddleSpikeGrab, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoMiddleSpikeScore, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoFarSpikeAlignment, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoFarSpikeGrab, false),
            PedroCommands.follow(AutoPoseUtil.follower, AutoPoseUtil.BlueDepoFarSpikeScore, false)
        )
    }

    fun initializePedroPathing() {
        Scheduler.reset()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(AutoPoseUtil.startPoseBlueDepoPose)
        AutoPoseUtil.follower = follower
    }
}