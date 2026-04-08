package org.firstinspires.ftc.teamcode.util

import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.subSystems.CamSS
import org.firstinspires.ftc.teamcode.subSystems.SpinDexerSS

/** A utility script to made display everything related to the robot in one file */
class PanelsDebugUtil (
    private val panels : TelemetryManager?
) {

    /** Calling this function will display all the telemetry related to the robot */
    fun showAllDebug(
        gamepad1 : Gamepad,
        centerUtil : CenterUtil,
        firing : FiringUtil,
        spinDexer : SpinDexerSS,
        cam : CamSS,
        hubUtil: HubUtil
    ) {
        panels?.apply {
            debug("=== GAMEPAD ===")
            debug("RB pressed:", gamepad1.right_bumper)
            debug("Square:", gamepad1.square)
            debug("Triangle:", gamepad1.triangle)
            debug("Circle:", gamepad1.circle)
            debug("Cross released:", gamepad1.crossWasReleased())
            debug("")

            debug("=== HUB ===")
            debug("Temperature Fahrenheit", hubUtil.getTempFahrenheit())
            debug("Temperature Celsius", hubUtil.getTempCelsius())
            debug("Temperature Kelvin", hubUtil.getTempKelvin())
            debug("")

            debug("=== STATE ===")
            debug("Is Centering:", centerUtil.isCentering())
            debug("Is Firing:", firing.isFiring())
            debug("")

            debug("=== POWER ===")
            debug("Center Power:", centerUtil.getRotationPower())
            debug("Fly Wheel Power:", firing.flyWheelPower())
            debug("")

            debug("=== SPINDEXER ===")
            debug("SpinDexer Position:", spinDexer.position())
            debug("SpinDexer Raw Position:", spinDexer.rawPosition())
            debug("")

            debug("=== CAM ===")
            debug("Cam Position", cam.position())
            debug("Cam Raw Position", cam.rawPosition())
        }
    }

    /** Calling this function will show an init message to confirm that init has not only started but this utility is working */
    fun showInit() {
        panels?.debug("Init Started")
    }

    /** Call this function while passing the telemetry object from the SDK to display everything to panels and the DS */
    fun update(telemetry: org.firstinspires.ftc.robotcore.external.Telemetry) {
        panels?.update(telemetry)
    }
}