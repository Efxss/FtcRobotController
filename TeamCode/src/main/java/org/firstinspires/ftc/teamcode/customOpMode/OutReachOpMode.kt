package org.firstinspires.ftc.teamcode.customOpMode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.util.HubUtil
import org.firstinspires.ftc.teamcode.util.PanelsDebugUtil

/**
 * Custom-made OpMode for the DECODE post season OutReach robot
 * @author Jonny Todd - 29403 PiBytes
 */
abstract class OutReachOpMode : OpMode() {

    // Shared resources
    private var panels: TelemetryManager? = null
    private lateinit var debugUtil : PanelsDebugUtil
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

        // declare panels and init the debug util
        panels = PanelsTelemetry.telemetry
        debugUtil = PanelsDebugUtil(panels)

        // init bulkRead
        hubUtil = HubUtil(hardwareMap)

        // Show the init message and update panels to display it
        debugUtil.showInit()
        debugUtil.update(telemetry)
        onInit()
    }

    final override fun init_loop() {
        // Clear the bulk read cache
        hubUtil.clearCache()
        onInitLoop()
    }

    final override fun start() {
        onStart()
    }

    final override fun loop() {
        hubUtil.clearCache()
        onLoop()
    }

    final override fun stop() {
        onStop()
    }

    // Custom functions

    /** Calling this function will return the panels variable to be accessible in TeleOP
     * @see [getDebugUtil]
     * @see [getHubUtil] */
    protected fun getPanels() = panels

    /** Calling this function will return the debugUtil variable to be accessible in TeleOP
     * @see [getPanels]
     * @see [getHubUtil] */
    protected fun getDebugUtil() = debugUtil

    /** Calling this function will return the bulkRead variable to be accessible in TeleOP
     * @see [getDebugUtil]
     * @see [getPanels] */
    protected fun getHubUtil() = hubUtil
}