package org.firstinspires.ftc.teamcode.opmode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

@TeleOp
class HeadingTest : TeleOpMode() {
    override val alliance = VariableStateUtil.alliance

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
        follower.setStartingPose(resetPose)
    }
}