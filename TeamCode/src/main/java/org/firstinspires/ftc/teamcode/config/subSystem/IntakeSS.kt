package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.config.Robot

class IntakeSS(
    robot: Robot
) {
    init {
        robot.intakeMotor.setRunMode(Motor.RunMode.VelocityControl)
        robot.intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        robot.intakeMotor.inverted = true
    }
    private val intakeVelocity = 0.35 // -980.0
    //private fun setVelocity(velocity: Double, robot: Robot) {
    //    robot.intakeMotor.set(velocity)
    //}
    val runIntakeCommand: Command = Command.build()
        //.setStart { setVelocity(intakeVelocity, robot) }
        //.setEnd { setVelocity(0.0, robot) }
        .setStart { robot.intakeMotor.set(intakeVelocity) }
        .setEnd { robot.intakeMotor.set(0.0) }
    val reverseIntakeCommand: Command = Command.build()
        //.setStart { setVelocity(-intakeVelocity, robot) }
        //.setEnd { setVelocity(0.0, robot) }
        .setStart { robot.intakeMotor.set(-intakeVelocity) }
        .setEnd { robot.intakeMotor.set(0.0) }
}