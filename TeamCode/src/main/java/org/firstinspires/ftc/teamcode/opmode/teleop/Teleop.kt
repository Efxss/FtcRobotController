package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.subSystem.RampSS
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

@Configurable
@TeleOp
class Teleop : TeleOpMode() {
    companion object {
        var intakePose: Double = 0.0
        var holdPose: Double = 0.25
        var firePos: Double = 0.5
        var rampState: RampSS.STATE = RampSS.STATE.INTAKE
    }
    override val alliance = VariableStateUtil.alliance

    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        //follower.startTeleopDrive()
    }

    override fun onLoop() {
        /*follower.update()
        if (gamepad1.leftBumperWasPressed() && llss.isTagSeen(alliance) && !isAutoTurning) {
            follower.turn(llss.currentTagXRad(alliance, autoTurnPixel))
            isAutoTurning = true
            autoTurnStartTime = runtime
            return
        }
        if (isAutoTurning) {
            val timedOut = (runtime - autoTurnStartTime) >= autoTurnTimeoutSec
            if (!follower.isTurning || timedOut) {
                gamepad1.rumble(1.0, 1.0, 150)
                gamepad1.setLedColor(0.0, 255.0, 0.0, 1000)
                follower.startTeleopDrive()
                isAutoTurning = false
            }
        } else {
            follower.setTeleOpDrive(forward, strafe, rotate, false)
        }*/
        rampSS.update(rampState, intakePose, holdPose, firePos)
    }

    fun initializePedroPathing() {
        val startPose = VariableStateUtil.endOfAutoPose ?: Pose()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
    }
}