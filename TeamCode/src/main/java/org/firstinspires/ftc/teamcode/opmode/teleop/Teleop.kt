package org.firstinspires.ftc.teamcode.opmode.teleop

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.VariableState

@TeleOp
class Teleop : TeleOpMode() {
    lateinit var follower: Follower
    lateinit var pathTimer: Timer
    lateinit var actionTimer: Timer
    lateinit var opmodeTimer: Timer
    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        opmodeTimer.resetTimer()
        follower.startTeleopDrive()
    }

    override fun onLoop() {
        follower.update()
        var rotate = if(gamepad1.left_bumper) 1.0 else if (gamepad1.right_bumper) -1.0 else 0.0
        var forward = -gamepad1.left_stick_y.toDouble()
        var strafe = gamepad1.right_stick_x.toDouble()
        follower.setTeleOpDrive(
            forward,
            strafe,
            rotate,
            true
        )
    }

    fun initializePedroPathing() {
        val startPose = VariableState.endOfAutoPose ?: Pose()
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
    }
}