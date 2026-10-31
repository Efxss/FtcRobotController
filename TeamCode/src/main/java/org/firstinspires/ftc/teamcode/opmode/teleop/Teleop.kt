package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode
import org.firstinspires.ftc.teamcode.config.util.StateUtil

@Configurable
@TeleOp
class Teleop : TeleOpMode() {
    companion object {
        @JvmField
        var privateHoodAngle: Double = 0.0
        @JvmField
        var privateWheelPower: Double = 0.0
    }
    override val alliance = StateUtil.alliance

    override fun onInit() {
    }

    override fun onStart() {
        firingSS.calcFiring().schedule()
    }

    override fun onLoop() {
        //robot.follower.manual(dp)
        //ManualDrive.driveOrHold(robot.follower, dp) // Do this to make it so when you stop holding the stick it holds position
        StateUtil.hoodAngle = privateHoodAngle
        StateUtil.wheelPower = privateWheelPower
        robot.follower.update()
    }
}