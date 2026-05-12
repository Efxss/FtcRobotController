package org.firstinspires.ftc.teamcode.opmode.teleop

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants

@TeleOp
class Teleop : TeleOpMode() {
    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        follower.startTeleopDrive()
    }

    override fun onLoop() {
        follower.update()
        var rotate = if(gamepad1.left_bumper) 1.0 else if (gamepad1.right_bumper) -1.0 else 0.0
        var forward = -gamepad1.left_stick_y.toDouble()
        var strafe = -gamepad1.right_stick_x.toDouble()
        follower.setTeleOpDrive(forward, strafe, rotate, true)
    }

    fun initializePedroPathing() {
        //val startPose = VariableStateUtil.endOfAutoPose ?: Pose()
        val startPose = Pose(8.5, 8.375, Math.toRadians(90.0))
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
    }
}