package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.seattlesolvers.solverslib.hardware.motors.Motor
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.util.MathUtil

class IntakeSS(
    private val robot: Robot
) {
    init {
        robot.intakeMG.setRunMode(Motor.RunMode.VelocityControl)
        robot.intakeMG.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE)
        //robot.intakeMG.inverted = true
        robot.intakeM1.inverted = true
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
            this.currentVel = robot.intakeM1.correctedVelocity
            this.pidfVelocity = robot.intakeMPIDF.calculate(this.currentVel, MathUtil.pidfPower(this.velocity))
            robot.intakeMG.set(this.pidfVelocity)
        }
        .setEnd { robot.intakeMG.stopMotor() }
        .requiring(robot.intakeMG)
    private fun setRev(state: Boolean) { if (this.rev != state) { this.rev = state }; robot.intakeM1.inverted = !this.rev; robot.intakeM2.inverted = this.rev }
    fun reverseIntake(state: Boolean) {setRev(state)}
}