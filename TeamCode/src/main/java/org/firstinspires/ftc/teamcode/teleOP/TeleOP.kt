package org.firstinspires.ftc.teamcode.teleOP

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.util.CenterUtil

@TeleOp(name = "TeleOP", group = "Main TeleOP")
class TeleOP : OpMode() {
    var panels: TelemetryManager? = null
    private lateinit var centerUtil: CenterUtil
    override fun init() {
        panels = PanelsTelemetry.telemetry
        centerUtil = CenterUtil(hardwareMap, 0.25, 15, 17)
        panels?.debug("Init Started")
        panels?.update(telemetry)
    }

    override fun loop() {
        centerUtil.centering(gamepad1.right_bumper)
        panels?.debug("Bumper", gamepad1.right_bumper)
        panels?.debug("Is Centering", centerUtil.isCentering())
        panels?.debug("Power", centerUtil.getPower())
        panels?.update(telemetry)
    }
}