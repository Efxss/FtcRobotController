package org.firstinspires.ftc.teamcode.opmode.auto.red

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.Alliance
import org.firstinspires.ftc.teamcode.config.util.AutoPoseUtil

@Autonomous(group = "Red Auto", name = "Auto Pod Path Test Red")
class RedAuto : AutoOpMode() {
    override val alliance = Alliance.RED
    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        Scheduler.schedule(
            Groups.sequential(
                Commands.waitMs(250.0),
                Groups.loop(getIntakeSS().runIntake)
            )
        )
        Scheduler.schedule(AutoPoseUtil.allSpikeAutoRed())
    }

    override fun onLoop() {
        follower.update()
        Scheduler.execute()
    }

    fun initializePedroPathing() {
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(AutoPoseUtil.startPoseRedDepoPose)
        AutoPoseUtil.follower = follower
    }
}