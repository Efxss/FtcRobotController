package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DigitalChannel

object IntakeUtil {

    fun apply(
        isFull: Boolean,
        intake: CRServo,
        ledR: DigitalChannel,
        ledG: DigitalChannel
    ) {
        if (isFull) {
            intake.power = Config.ServoPositions.INTAKE_REVERSE
            setBothLed(ledR, ledG, on = true)
        } else {
            intake.power = Config.ServoPositions.INTAKE_ON
            setBothLed(ledR, ledG, on = false)
        }
    }

    private fun setBothLed(ledR: DigitalChannel, ledG: DigitalChannel, on: Boolean) {
        // Your wiring: state=false => ON, state=true => OFF
        val state = !on
        ledR.state = state
        ledG.state = state
    }
}
