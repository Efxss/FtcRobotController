package org.firstinspires.ftc.teamcode.config.customOpMode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.config.util.DrawingUtil
import org.firstinspires.ftc.teamcode.config.util.HubUtil

/**
 * Custom-made OpMode to copy and make a real OpMode
 * @author Jonny Todd - 29403 PiBytes
 */
abstract class TeleOpMode : OpMode() {

    // Shared resources
    private var panels : TelemetryManager? = null
    private lateinit var follower: Follower
    private lateinit var hubUtil : HubUtil

    // Custom lifecycle hooks

    /**
     * Mandatory function that will run all code inside one time upon pressing the initialization button
     */
    abstract fun onInit()
    /**
     * Mandatory function that will run all code inside continuously upon pressing the start button
     */
    abstract fun onLoop()

    /**
     * Optional function that will run all code inside continuously upon pressing the initialization button until start button is pressed
     */
    open fun onInitLoop() {}

    /**
     * Optional function that will run all code inside one time upon pressing the start button
     */
    open fun onStart() {}
    /**
     * Optional function that will run all code inside one time upon pressing the stop button
     */
    open fun onStop() {}

    final override fun init() {
        // Any other shared init (hardware caching, subsystems, etc.)

        // Declare panels and init it
        panels = PanelsTelemetry.telemetry

        // Init the drawing util for panels and PedroPathing
        DrawingUtil.init()

        // Init bulkRead
        hubUtil = HubUtil(hardwareMap)

        onInit()
    }

    final override fun init_loop() {
        // Draw on panels
        DrawingUtil.drawOnlyCurrent(follower)
        // Clear the bulk read cache
        hubUtil.clearCache()
        onInitLoop()
    }

    final override fun start() {
        onStart()
    }

    final override fun loop() {
        // Draw on panels
        DrawingUtil.drawDebug(follower)
        // Clear the bulk read cache
        hubUtil.clearCache()
        onLoop()
    }

    final override fun stop() {
        onStop()
    }

    // Custom functions

    /** Calling this function will return the panels variable to be accessible in TeleOP
     * @see [getHubUtil] */
    protected fun getPanels() = panels

    /** Calling this function will return the bulkRead variable to be accessible in TeleOP
     * @see [getPanels] */
    protected fun getHubUtil() = hubUtil
}
