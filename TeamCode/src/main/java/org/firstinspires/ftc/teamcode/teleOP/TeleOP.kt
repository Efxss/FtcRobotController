package org.firstinspires.ftc.teamcode.teleOP

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.subSystems.CamSS
import org.firstinspires.ftc.teamcode.subSystems.SpinDexerSS
import org.firstinspires.ftc.teamcode.util.CenterUtil
import org.firstinspires.ftc.teamcode.util.FiringUtil
import org.firstinspires.ftc.teamcode.util.PanelsDebugUtil

@TeleOp(name = "TeleOP", group = "Main TeleOP")
class TeleOP : OpMode() {
    private var panels: TelemetryManager? = null
    private lateinit var debugUtil : PanelsDebugUtil
    private lateinit var centerUtil : CenterUtil
    private lateinit var spinDexer : SpinDexerSS
    private lateinit var firing : FiringUtil
    private lateinit var cam : CamSS

    override fun init() {
        panels = PanelsTelemetry.telemetry
        debugUtil = PanelsDebugUtil(panels)

        centerUtil = CenterUtil(hardwareMap, 0.25, 15, 17)
        cam = CamSS(hardwareMap, 0.5 , 0.0)
        spinDexer = SpinDexerSS(hardwareMap)
        firing = FiringUtil(hardwareMap, spinDexer, cam, 1.0)

        spinDexer.loadOne(true)

        debugUtil.showInit()
        debugUtil.update(telemetry)
    }

    override fun loop() {
        centerUtil.centering(gamepad1.right_bumper)

        spinDexer.apply {
            loadOne(gamepad1.square)
            loadTwo(gamepad1.triangle)
            loadThree(gamepad1.circle)
        }

        firing.apply {
            startFiring(gamepad1.crossWasReleased())
            update()
        }

        debugUtil.showAllDebug(
            gamepad1 = gamepad1,
            centerUtil = centerUtil,
            firing = firing,
            spinDexer = spinDexer,
            cam = cam
        )
        debugUtil.update(telemetry)
    }

    override fun stop() {
        centerUtil.close()
    }
}