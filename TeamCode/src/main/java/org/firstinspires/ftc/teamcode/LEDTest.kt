package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel

@TeleOp
class LEDTest : OpMode() {
    private lateinit var statusLEDR: DigitalChannel
    private lateinit var statusLEDG: DigitalChannel
    override fun init() {
        statusLEDR = hardwareMap.get(DigitalChannel::class.java, "statusLEDR")
        statusLEDG = hardwareMap.get(DigitalChannel::class.java, "statusLEDG")
        statusLEDR.mode = DigitalChannel.Mode.OUTPUT
        statusLEDG.mode = DigitalChannel.Mode.OUTPUT
    }

    override fun loop() {
        if (gamepad1.cross) {
            statusLEDR.state = false // On
        }

        if (gamepad1.circle) {
            statusLEDR.state = true // Off
        }

        if (gamepad1.triangle) {
            statusLEDG.state = false // On
        }

        if (gamepad1.square) {
            statusLEDG.state = true // Off
        }

        telemetry.update()
    }
}