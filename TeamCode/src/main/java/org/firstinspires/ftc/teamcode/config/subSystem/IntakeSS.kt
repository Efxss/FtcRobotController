package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.config.Robot

class IntakeSS(
    robot: Robot
) {
    init {
        robot.intakeMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        robot.intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    private val intakeVelocity = -980.0 // -980.0
    private var lastVelocity: Double = Double.NaN
    private fun setVelocity(velocity: Double, robot: Robot) { if (velocity != lastVelocity) { robot.intakeMotor.velocity = velocity
            lastVelocity = velocity } }
    val runIntakeCommand: Command = Command.build()
        .setStart { setVelocity(intakeVelocity, robot) }
        .setEnd { setVelocity(0.0, robot) }
    val reverseIntakeCommand: Command = Command.build()
        .setStart { setVelocity(-intakeVelocity, robot) }
        .setEnd { setVelocity(0.0, robot) }
}