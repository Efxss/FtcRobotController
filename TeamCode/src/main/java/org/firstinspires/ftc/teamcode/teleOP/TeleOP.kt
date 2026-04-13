package org.firstinspires.ftc.teamcode.teleOP

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.customOpMode.OutReachOpMode
import org.firstinspires.ftc.teamcode.subSystems.CamSS
import org.firstinspires.ftc.teamcode.subSystems.SpinDexerSS
import org.firstinspires.ftc.teamcode.util.CenterUtil
import org.firstinspires.ftc.teamcode.util.FiringUtil

@TeleOp(name = "TeleOP", group = "Main TeleOP")
class TeleOP : OutReachOpMode() {
    private lateinit var centerUtil : CenterUtil
    private lateinit var spinDexer : SpinDexerSS
    private lateinit var firing : FiringUtil
    private lateinit var cam : CamSS
    private val pidf = PIDFCoefficients(150.0, 0.0, 0.0, 13.5)

    override fun onInit() {
        centerUtil = CenterUtil(hardwareMap, 0.25, 15, 17)
        cam = CamSS(hardwareMap, 0.5 , 0.0)
        spinDexer = SpinDexerSS(hardwareMap)
        firing = FiringUtil(hardwareMap, spinDexer, cam, 0.6, 0.85, pidf)

        spinDexer.loadOne(true)
    }

    override fun onLoop() {
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

        getDebugUtil().showAllDebug(
            gamepad1 = gamepad1,
            centerUtil = centerUtil,
            firing = firing,
            spinDexer = spinDexer,
            cam = cam,
            hubUtil = getHubUtil()
        )
        getDebugUtil().update(telemetry)
    }

    override fun onStop() {
        centerUtil.close()
    }
}