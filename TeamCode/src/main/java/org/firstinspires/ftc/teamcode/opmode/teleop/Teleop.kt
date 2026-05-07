package org.firstinspires.ftc.teamcode.opmode.teleop

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.ExampleOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants

@TeleOp
class Teleop : ExampleOpMode() {
    val startPose = Pose(72.0, 8.375, Math.toRadians(0.0))
    lateinit var follower: Follower
    lateinit var pathTimer: Timer
    lateinit var actionTimer: Timer
    lateinit var opmodeTimer: Timer
    override fun onInit() {
        initializePedroPathing()
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
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
    }
}