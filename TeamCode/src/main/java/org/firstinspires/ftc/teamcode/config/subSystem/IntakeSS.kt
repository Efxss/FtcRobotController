package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.config.Robot

class IntakeSS(
    robot: Robot
) {
    init {
        robot.intakeM.setRunMode(Motor.RunMode.VelocityControl)
        robot.intakeM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        robot.intakeM.inverted = true
    }
    private val intakeVelocity = 0.3
    val runIntake: Command = Command.build()
        .setStart { robot.intakeM.set(intakeVelocity) }
        .setEnd { robot.intakeM.stopMotor() }
        .requiring(robot.intakeM)
    val reverseIntake: Command = Command.build()
        .setStart { robot.intakeM.set(-intakeVelocity) }
        .setEnd { robot.intakeM.stopMotor() }
        .requiring(robot.intakeM)
}