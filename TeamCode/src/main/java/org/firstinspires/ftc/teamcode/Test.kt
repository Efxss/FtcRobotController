package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode

class Test : OpMode() {
    var panels: TelemetryManager? = null
    override fun init() {
        panels?.debug("Test 1")
        panels?.update(telemetry)
    }

    override fun loop() {
        panels?.debug("Test 2")
        panels?.update(telemetry)
    }
}