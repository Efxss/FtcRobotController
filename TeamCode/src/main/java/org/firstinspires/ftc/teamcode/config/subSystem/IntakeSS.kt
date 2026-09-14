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
    private val intakeVelocity = 0.3
    val runIntakeCommand: Command = Command.build()
        .setStart { robot.intakeMotor.set(intakeVelocity) }
        .setEnd { robot.intakeMotor.motorEx.power = 0.0 }
    val reverseIntakeCommand: Command = Command.build()
        .setStart { robot.intakeMotor.set(-intakeVelocity) }
        .setEnd { robot.intakeMotor.motorEx.power = 0.0 }
}