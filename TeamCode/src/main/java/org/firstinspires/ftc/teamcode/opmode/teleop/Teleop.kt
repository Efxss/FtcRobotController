package org.firstinspires.ftc.teamcode.opmode.teleop

import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil
import kotlin.math.abs

@TeleOp
class Teleop : TeleOpMode() {
    override val alliance = VariableStateUtil.alliance
    lateinit var aprilTagAlignPathChain : PathChain
    private var isAutoTurning = false
    private var autoTurnStartTime = 0.0
    private val autoTurnDoneRad = Math.toRadians(2.0)
    private val autoTurnTimeoutSec = 1.5
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
        val rotate = gamepad1.right_stick_x.toDouble()
        val forward = -gamepad1.left_stick_y.toDouble()
        val strafe  = -gamepad1.left_stick_x.toDouble()
        if (gamepad1.rightBumperWasPressed() && llss.isTagSeen(alliance) && !isAutoTurning) {
            follower.turn(Math.toRadians(llss.currentTagXDeg(alliance)))
            isAutoTurning = true
            autoTurnStartTime = runtime
        }
        if (isAutoTurning) {
            val headingErr = abs(follower.headingError)
            val timedOut = (runtime - autoTurnStartTime) >= autoTurnTimeoutSec
            if (headingErr <= autoTurnDoneRad || timedOut) {
                follower.startTeleopDrive()
                isAutoTurning = false
            }
        } else {
            follower.setTeleOpDrive(forward, strafe, rotate, true)
        }
        if (gamepad1.circle) getIntakeSS().runIntakeFun()
        else getIntakeSS().stopIntakeFun()
        if (gamepad1.cross) follower.pose = resetPose
    }

    fun initializePedroPathing() {
        val startPose = VariableStateUtil.endOfAutoPose ?: Pose()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
    }
}