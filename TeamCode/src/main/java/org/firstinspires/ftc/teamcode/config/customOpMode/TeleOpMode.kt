package org.firstinspires.ftc.teamcode.config.customOpMode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.config.subSystem.IntakeSS
import org.firstinspires.ftc.teamcode.config.util.Alliance
import org.firstinspires.ftc.teamcode.config.util.DrawingUtil
import org.firstinspires.ftc.teamcode.config.util.HubUtil
import org.firstinspires.ftc.teamcode.config.util.PanelsDebugUtil

/**
 * Custom-made OpMode to copy and make a real OpMode
 * @author Jonny Todd - 29403 PiBytes
 */
abstract class TeleOpMode : OpMode() {

    // Shared resources
    private var panels : TelemetryManager? = null
    private lateinit var hubUtil : HubUtil
    private lateinit var debugUtil : PanelsDebugUtil
    private lateinit var intakeSS : IntakeSS
    protected lateinit var follower : Follower
    protected var resetPose = Pose(8.0, 8.0, Math.toRadians(90.0))


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
        debugUtil.update(telemetry)
        intakeSS = IntakeSS(hardwareMap)
        hubUtil = HubUtil(hardwareMap)
        onInit()
    }

    final override fun init_loop() {
        // Clear the bulk read cache
        hubUtil.clearCache()
        // Draw on Panels
        DrawingUtil.drawOnlyCurrent(follower)
        onInitLoop()
    }

    final override fun start() {
        resetRuntime()
        resetPose = when (alliance) {
            Alliance.BLUE -> {
                Pose(8.0, 8.0, Math.toRadians(90.0))
            }

            Alliance.RED -> {
                Pose(136.0, 8.0, Math.toRadians(90.0))
            }
        }
        onStart()
    }

    final override fun loop() {
        // Clear the bulk read cache
        hubUtil.clearCache()
        // Draw on Panels
        DrawingUtil.drawDebug(follower)
        //Show and update debug
        debugUtil.showAllDebugTeleop(follower, hubUtil, alliance, runtime, gamepad1)
        debugUtil.update(telemetry)
        onLoop()
    }

    final override fun stop() {
        onStop()
    }

    // Custom functions

    /** Calling this function will return the Panels variable to be accessible in Teleop
     * @see [getDebugUtil]
     * @see [getHubUtil]
     * @see [getIntakeSS]
     */
    protected fun getPanels() = panels

    /** Calling this function will return the DebugUtil variable to be accessible in Teleop
     * @see [getPanels]
     * @see [getHubUtil]
     * @see [getIntakeSS]
     */
    protected fun getDebugUtil() = debugUtil

    /** Calling this function will return the HubUtil variable to be accessible in Teleop
     * @see [getDebugUtil]
     * @see [getPanels]
     * @see [getIntakeSS]
     */
    protected fun getHubUtil() = hubUtil

    /** Calling this function will return the Intake SubSystem to be accessible in Teleop
     * @see [getPanels]
     * @see [getDebugUtil]
     * @see [getHubUtil]
     */
    protected fun getIntakeSS() = intakeSS
}
