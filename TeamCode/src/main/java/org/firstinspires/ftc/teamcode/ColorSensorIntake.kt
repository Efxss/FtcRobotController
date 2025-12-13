package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class ColorSensorIntake : OpMode() {
    var panels: TelemetryManager? = null
    private lateinit var colorSensor: ColorSensor
    private lateinit var bowlServo: Servo
    var ord = arrayOf("N", "N", "N")
    var isSeen = false
    object ServoPositions {
        const val LOAD_P1 = 0.0
        const val LOAD_P2 = 0.075
        const val LOAD_P3 = 0.148
    }
    override fun init() {
        colorSensor = hardwareMap.get(ColorSensor::class.java, "ColorSensor")
        bowlServo = hardwareMap.get(Servo::class.java, "bowlServo")
        bowlServo.position = ServoPositions.LOAD_P1
        panels = PanelsTelemetry.telemetry
    }

    override fun loop() {
        var r = colorSensor.red();var g = colorSensor.green();var b = colorSensor.blue()
        panels?.debug("Red", r)
        panels?.debug("Green", g)
        panels?.debug("Blue", b)
        if (g <= 85 && b <= 110) {
            panels?.debug("None")
            isSeen = false
        }
        if (g >= 85 && b >= 110) {
            panels?.debug("Purple")
            if (!isSeen) {
                val slot = nextSlot()
                if (slot != -1) {
                    ord[slot] = "P"
                    advanceBowl(slot)
                }
                isSeen = true
            }
        }
        if (g >= 100) {
            panels?.debug("Green")
            if (!isSeen) {
                val slot = nextSlot()
                if (slot != -1) {
                    ord[slot] = "G"
                    advanceBowl(slot)
                }
                isSeen = true
            }
        }
        panels?.debug("order 1", ord[0])
        panels?.debug("order 2", ord[1])
        panels?.debug("order 3", ord[2])
        panels?.update(telemetry)
    }
    private fun nextSlot(): Int {
        return when {
            ord[0] == "N" -> 0
            ord[1] == "N" -> 1
            ord[2] == "N" -> 2
            else -> -1 // Array is full
        }
    }
    private fun advanceBowl(slot: Int) {
        bowlServo.position = when (slot) {
            0 -> ServoPositions.LOAD_P2
            1 -> ServoPositions.LOAD_P3
            else -> bowlServo.position // Stay in place if full
        }
    }
}