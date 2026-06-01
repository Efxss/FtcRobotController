package org.firstinspires.ftc.teamcode.opmode.teleop

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

@TeleOp
class Teleop : TeleOpMode() {
    override val alliance = VariableStateUtil.alliance

    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        //Scheduler.schedule(getIntakeSS().runIntakeCommand)
        follower.startTeleopDrive()
    }

    override fun onLoop() {
        follower.update()
        //Scheduler.execute()
        if (gamepad1.rightBumperWasPressed() && llss.isTagSeen(alliance) && !isAutoTurning) {
            follower.turn(llss.currentTagXRad(alliance, autoTurnPixel))
            isAutoTurning = true
            autoTurnStartTime = runtime
            return
        }
        if (isAutoTurning) {
            val timedOut = (runtime - autoTurnStartTime) >= autoTurnTimeoutSec
            if (!follower.isTurning || timedOut) {
                gamepad1.rumble(1.0, 1.0, 150)
                follower.startTeleopDrive()
                isAutoTurning = false
            }
        } else {
            follower.setTeleOpDrive(forward, strafe, rotate, false)
        }
    }

    fun initializePedroPathing() {
        val startPose = VariableStateUtil.endOfAutoPose ?: Pose()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
    }
}