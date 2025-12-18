package org.firstinspires.ftc.teamcode.OldCode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Servo
import kotlin.concurrent.thread

@Disabled
class ServoTest : OpMode() {
    var panels: TelemetryManager? = null
    lateinit var servo : Servo
    companion object {
        @JvmField
        var position = 0.toDouble()
    }
    override fun init() {
        servo = hardwareMap.get(Servo::class.java, "bowlServo")
        panels = PanelsTelemetry.telemetry
        var t = thread(start = true, name = "ServoTelemetryThread") {
            while (!Thread.currentThread().isInterrupted) {
                panels?.apply {
                    addLine("Servo Position: ${"%.2f".format(servo.position)}")
                    update()
                }
            }
        }
    }

    override fun loop() {
        val lP1 = 0.059
        val lP2 = 0.13
        val lP3 = 0.204
        val fP1 = 0.02
        val fP2 = 0.0945
        val fP3 = 0.167
        servo.position = position // 0.1135 seems good
    }

    override fun stop() {
        Thread.currentThread().interrupt()
        super.stop()
    }
}