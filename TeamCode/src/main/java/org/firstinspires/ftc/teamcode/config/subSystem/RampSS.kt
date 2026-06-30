package org.firstinspires.ftc.teamcode.config.subSystem

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class RampSS(
    hardwareMap: HardwareMap
)  {
    enum class STATE { INTAKE, HOLD, FIRE }
    private var lastPos: Double = Double.NaN
    val rampServo: Servo = hardwareMap.get(Servo::class.java, "ramp")
    fun update(state: STATE) {
        when (state) {
            STATE.INTAKE -> setPosition(0.0)
            STATE.HOLD -> setPosition(0.25)
            STATE.FIRE -> setPosition(0.5)
        }
    }
    private fun setPosition(pos: Double) { if (lastPos != rampServo.position) { rampServo.position = pos
            lastPos = pos } }
    fun position(): Double { return rampServo.position }
}