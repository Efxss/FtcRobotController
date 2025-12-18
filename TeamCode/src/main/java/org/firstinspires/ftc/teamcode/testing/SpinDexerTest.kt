package org.firstinspires.ftc.teamcode.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@Configurable
@TeleOp
class SpinDexerTest : OpMode() {
    private lateinit var bowlServo: Servo
    var panels: TelemetryManager? = null
    object ServoPositions {
        // Loading positions
        const val LOAD_P1 = 0.0
        const val LOAD_P2 = 0.45
        const val LOAD_P3 = 0.9

        // Firing/dispensing positions
        const val FIRE_P1 = 0.68
        const val FIRE_P2 = 0.1845
        const val FIRE_P3 = 0.258

        // Camera servo positions
        const val CAM_OPEN = 0.5
        const val CAM_CLOSED = 0.255

        const val INTAKE_ON = 1.0
        const val INTAKE_REVERSE = -1.0
        const val INTAKE_OFF = 0.0
    }
    companion object {
        @JvmField
        /*var lP1 = false
        var lP2 = false
        var lP3 = false
        var fP1 = false
        var fP2 = false
        var fP3 = false*/
        var mP = 0.toDouble()
    }
    override fun init() {panels = PanelsTelemetry.telemetry; bowlServo = hardwareMap.get(Servo::class.java, "bowlServo") }

    override fun loop() {
        /*if (lP1) {
            bowlServo.position = ServoPositions.LOAD_P1
            resetFlags()
        }
        if (lP2) {
            bowlServo.position = ServoPositions.LOAD_P2
            resetFlags()
        }
        if (lP3) {
            bowlServo.position = ServoPositions.LOAD_P3
            resetFlags()
        }
        if (fP1) {
            bowlServo.position = ServoPositions.FIRE_P1
            resetFlags()
        }
        if (fP2) {
            bowlServo.position = ServoPositions.FIRE_P2
            resetFlags()
        }
        if (fP3) {
            bowlServo.position = ServoPositions.FIRE_P3
            resetFlags()
        }*/
        bowlServo.position = mP
    }
    fun resetFlags() {
        //lP1 = false;lP2 = false;lP3 = false;fP1 = false;fP2 = false;fP3 = false
    }
}