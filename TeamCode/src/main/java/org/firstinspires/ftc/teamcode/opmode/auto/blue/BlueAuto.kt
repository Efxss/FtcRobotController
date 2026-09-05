package org.firstinspires.ftc.teamcode.opmode.auto.blue

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.groups.Groups
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants

@Autonomous(group = "Auto", name = "Auto")
class BlueAuto : AutoOpMode() {
    override val alliance = Robot.Alliance.BLUE
    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        intakeSS.runIntakeCommand.schedule()
        Scheduler.schedule(
            Groups.sequential(
                runAuto()
            )
        )
    }

    override fun onLoop() {
        follower.update()
    }

    fun initializePedroPathing() {
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Robot.AutoPoseUtil.startPose)
        Robot.AutoPoseUtil.follower = follower
    }
}