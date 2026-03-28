package org.firstinspires.ftc.teamcode.teleOP

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subSystems.SpinDexerSS
import org.firstinspires.ftc.teamcode.util.CenterUtil
import org.firstinspires.ftc.teamcode.util.FiringUtil

@TeleOp(name = "TeleOP", group = "Main TeleOP")
class TeleOP : OpMode() {
    var panels: TelemetryManager? = null
    private lateinit var centerUtil: CenterUtil
    private lateinit var spinDexer: SpinDexerSS
    private lateinit var firing: FiringUtil
    override fun init() {
        panels = PanelsTelemetry.telemetry
        centerUtil = CenterUtil(hardwareMap, 0.25, 15, 17)
        spinDexer = SpinDexerSS(hardwareMap)
        firing = FiringUtil(hardwareMap)
        panels?.debug("Init Started")
        panels?.update(telemetry)
    }

    override fun loop() {
        centerUtil.centering(gamepad1.right_bumper)
        spinDexer.apply {
            loadOne(gamepad1.square)
            loadTwo(gamepad1.triangle)
            loadThree(gamepad1.circle)
        }
        firing.executeFiring(gamepad1.crossWasReleased())
        panels?.debug("Bumper", gamepad1.right_bumper)
        panels?.debug("Is Centering", centerUtil.isCentering())
        panels?.debug("Power", centerUtil.getPower())
        panels?.update(telemetry)
    }
}