package org.firstinspires.ftc.teamcode.opmode.teleop

import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.groups.Groups
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.MathUtil
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

@TeleOp
class Teleop : TeleOpMode() {
    override val alliance = VariableStateUtil.alliance

    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        Scheduler.schedule(Groups.loop(getIntakeSS().runIntake))
        follower.startTeleopDrive()
    }

    override fun onLoop() {
        follower.update()
        Scheduler.execute()
        val rotate = if(gamepad1.left_bumper) 1.0 else if (gamepad1.right_bumper) -1.0 else 0.0
        val forward = MathUtil.shapeStick(-gamepad1.left_stick_y.toDouble())
        val strafe  = MathUtil.shapeStick(-gamepad1.right_stick_x.toDouble())
        follower.setTeleOpDrive(forward, strafe, rotate, true)
    }

    fun initializePedroPathing() {
        val startPose = VariableStateUtil.endOfAutoPose ?: Pose()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
    }
}