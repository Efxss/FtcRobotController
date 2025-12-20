package org.firstinspires.ftc.teamcode.testing

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.ColorSensor

@Disabled
class ColorSensorTest : OpMode() {
    var panels: TelemetryManager? = null
    private lateinit var colorSensor: ColorSensor
    override fun init() {
        colorSensor = hardwareMap.get(ColorSensor::class.java, "ColorSensor")
        panels = PanelsTelemetry.telemetry
    }

    override fun loop() {
        var r=colorSensor.red();var g=colorSensor.green();var b=colorSensor.blue()
        panels?.apply { debug("R", r);debug("G", g);debug("B", b);update(telemetry) }
    }
}