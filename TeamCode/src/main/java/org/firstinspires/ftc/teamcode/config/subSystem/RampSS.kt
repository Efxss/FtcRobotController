package org.firstinspires.ftc.teamcode.config.subSystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class RampSS(
    hardwareMap: HardwareMap
)  {
    enum class STATE { INTAKE, HOLD, FIRE }
    val rampServo: Servo = hardwareMap.get(Servo::class.java, "ramp")
    fun update(state: STATE) {
        when (state) {
            STATE.INTAKE -> rampServo.position = (0.0)
            STATE.HOLD -> rampServo.position = (0.34)
            STATE.FIRE -> rampServo.position = (0.53)
        }
    }
    fun position(): Double = rampServo.position
}