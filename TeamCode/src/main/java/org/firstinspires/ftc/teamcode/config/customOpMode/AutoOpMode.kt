package org.firstinspires.ftc.teamcode.config.customOpMode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.config.util.Alliance
import org.firstinspires.ftc.teamcode.config.util.DrawingUtil
import org.firstinspires.ftc.teamcode.config.util.HubUtil
import org.firstinspires.ftc.teamcode.config.util.PanelsDebugUtil
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

/**
 * Custom-made OpMode to copy and make a real OpMode
 * @author Jonny Todd - 29403 PiBytes
 */
abstract class AutoOpMode : OpMode() {

    // Shared resources
    private var panels : TelemetryManager? = null
    private lateinit var follower: Follower
    private lateinit var hubUtil : HubUtil
    private lateinit var debugUtil : PanelsDebugUtil

    // Custom lifecycle hooks

    /**
     * Mandatory property that defines which alliance this auto runs for.
     * Must be overridden by the subclass (e.g. `override val alliance = Alliance.BLUE`)
     */
    abstract val alliance: Alliance

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

        // declare Panels and init the debug util
        panels = PanelsTelemetry.telemetry
        debugUtil = PanelsDebugUtil(panels)
        debugUtil.showInit()

        // Init the drawing util for panels and PedroPathing
        DrawingUtil.init()

        // Reset Ivy scheduler so commands from a previous OpMode don't carry over
        Scheduler.reset()

        // Init bulkRead
        hubUtil = HubUtil(hardwareMap)
        debugUtil.update(telemetry)
        onInit()
    }

    final override fun init_loop() {
        // Draw on Panels
        DrawingUtil.drawOnlyCurrent(follower)
        // Clear the bulk read cache
        hubUtil.clearCache()
        onInitLoop()
    }

    final override fun start() {
        resetRuntime()
        onStart()
    }

    final override fun loop() {
        // Draw on Panels
        DrawingUtil.drawDebug(follower)
        //Show and update debug
        debugUtil.showAllDebug(follower, hubUtil, runtime)
        debugUtil.update(telemetry)
        // Update PedroPathing
        follower.update()
        // Clear the bulk read cache
        hubUtil.clearCache()
        onLoop()
    }

    final override fun stop() {
        VariableStateUtil.endOfAutoPose = follower.pose
        onStop()
    }

    // Custom functions

    /** Calling this function will return the Panels variable to be accessible in Auto
     * @see [getDebugUtil]
     * @see [getHubUtil]
     * @see [getFollower]
     */
    protected fun getPanels() = panels

    /** Calling this function will return the DebugUtil variable to be accessible in Auto
     * @see [getPanels]
     * @see [getHubUtil]
     * @see [getFollower]
     */
    protected fun getDebugUtil() = debugUtil

    /** Calling this function will return the HubUtil variable to be accessible in Auto
     * @see [getDebugUtil]
     * @see [getPanels]
     * @see [getFollower]
     */
    protected fun getHubUtil() = hubUtil


    /** Calling this function will return the Follower variable to be accessible in Auto
     * @see [getPanels]
     * @see[getDebugUtil]
     * @see [getHubUtil]
     */
    protected fun getFollower() = follower
}