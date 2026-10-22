package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.util.MathUtil

class IntakeSS(
    private val robot: Robot
) {
    init {
        robot.intakeM.setRunMode(Motor.RunMode.VelocityControl)
        robot.intakeM.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        robot.intakeM.inverted = true
    }

    /**
     * The Velocity of the intake like what did you think it would be???
     */
    var velocity: Double = robot.intakeDef
    private var currentVel: Double = 0.0
    private var pidfVelocity: Double = 0.0
    private var rev = true
    fun runIntake(): Command = Command.build()
        .setExecute {
            this.currentVel = robot.intakeM.correctedVelocity
            this.pidfVelocity = robot.intakeMPIDF.calculate(this.currentVel, MathUtil.pidfPower(this.velocity))
            robot.intakeM.set(this.pidfVelocity)
        }
        .setEnd { robot.intakeM.stopMotor() }
        .requiring(robot.intakeM)
    private fun setRev(state: Boolean) {if (this.rev != state) this.rev = state; robot.intakeM.inverted = !this.rev}
    fun reverseIntake(state: Boolean) {setRev(state)}
}