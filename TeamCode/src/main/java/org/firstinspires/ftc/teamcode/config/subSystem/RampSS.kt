package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

class RampSS(
    hardwareMap: HardwareMap
) {
    enum class STATE { INTAKE, HOLD, FIRE }
    private val rampServo: Servo = hardwareMap.get(Servo::class.java, "ramp")
    fun update(state: STATE) {
        when (state) {
            STATE.INTAKE -> rampServo.position = (0.0)
            STATE.HOLD -> rampServo.position = (0.34)
            STATE.FIRE -> rampServo.position = (0.53)
        }
    }
    fun rampIntake(): Command { return instant { VariableStateUtil.rampState = RampSS.STATE.INTAKE } }
    fun rampHold(): Command { return  instant { VariableStateUtil.rampState = RampSS.STATE.HOLD }}
    fun rampFire(): Command { return instant { VariableStateUtil.rampState = RampSS.STATE.FIRE } }
    fun position(): Double = rampServo.position
}