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
    private val intakeVelocity = 0.6
    private var rev = true
    fun runIntake(): Command = Command.build()
        .setExecute { robot.intakeM.set(intakeVelocity) }
        .setEnd { robot.intakeM.stopMotor() }
        .requiring(robot.intakeM)
    private fun setRev(state: Boolean) {if (rev != state) rev = state; robot.intakeM.inverted = !rev}
    fun reverseIntake(state: Boolean) {setRev(state)}
}