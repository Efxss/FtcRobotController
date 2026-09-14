package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

@Configurable
@TeleOp
class Teleop : TeleOpMode() {
    override val alliance = VariableStateUtil.alliance

    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        follower.startTeleopDrive()
    }

    override fun onLoop() {
        follower.update()
    }

    fun initializePedroPathing() {
        val startPose = VariableStateUtil.endOfAutoPose ?: Pose()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
    }
}