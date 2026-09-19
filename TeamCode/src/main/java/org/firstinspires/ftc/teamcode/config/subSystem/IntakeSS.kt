package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.config.Robot

class IntakeSS(
    private val robot: Robot
) {
    init {
        robot.intakeM.setRunMode(Motor.RunMode.VelocityControl)
        robot.intakeM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        robot.intakeM.inverted = true
    }
    private val intakeVelocity = 0.3
    fun runIntake(): Command = Command.build()
        .setExecute { robot.intakeM.set(intakeVelocity) }
        .setEnd { robot.intakeM.stopMotor() }
        .setPriority(1)
        .requiring(robot.intakeM)
    fun reverseIntake(): Command = Command.build()
        .setExecute { robot.intakeM.set(-intakeVelocity) }
        .setEnd { robot.intakeM.stopMotor() }
        .setPriority(1)
        .requiring(robot.intakeM)
}