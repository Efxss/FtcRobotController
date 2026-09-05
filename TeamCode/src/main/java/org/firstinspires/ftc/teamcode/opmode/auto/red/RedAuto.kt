package org.firstinspires.ftc.teamcode.opmode.auto.red

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants

@Disabled
//@Autonomous(group = "Red Auto", name = "Red Auto")
class RedAuto : AutoOpMode() {
    override val alliance = Robot.Alliance.RED
    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
    }

    override fun onLoop() {
        follower.update()
    }

    fun initializePedroPathing() {
        follower = Constants.createFollower(hardwareMap)
        Robot.AutoPoseUtil.follower = follower
    }
}