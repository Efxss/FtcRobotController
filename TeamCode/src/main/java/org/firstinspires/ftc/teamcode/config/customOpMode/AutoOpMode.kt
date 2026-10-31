package org.firstinspires.ftc.teamcode.config.customOpMode

import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.subSystem.IntakeSS
import org.firstinspires.ftc.teamcode.config.util.DrawingUtil
import org.firstinspires.ftc.teamcode.config.subSystem.HubSS
import org.firstinspires.ftc.teamcode.config.util.PanelsDebugUtil
import org.firstinspires.ftc.teamcode.config.util.StateUtil

/**
 * Custom-made OpMode for making an Auto that has needed code for a OpMode
 * @author Jonny Todd - 29403 PiBytes
 */
abstract class AutoOpMode : OpMode() {

    // Shared resources
    protected lateinit var robot: Robot
    protected lateinit var hubSS: HubSS
    protected lateinit var debugUtil: PanelsDebugUtil
    protected lateinit var intakeSS: IntakeSS

    // Custom lifecycle hooks

    /**
     * Mandatory property that defines which alliance this auto runs for.
     * Must be overridden by the subclass (e.g. `override val alliance = Alliance.BLUE`)
     */
    abstract val alliance: Robot.Alliance

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
        robot = Robot(hardwareMap)

        // declare Panels and init the debug util
        robot.initPanels()
        debugUtil = PanelsDebugUtil(robot.panels)
        debugUtil.showInit()

        // Reset Ivy scheduler so commands from a previous OpMode don't carry over
        Scheduler.reset()

        // Init bulkRead
        debugUtil.update(telemetry)
        intakeSS = IntakeSS(robot)
        hubSS = HubSS(hardwareMap)
        onInit()
    }

    final override fun init_loop() {
        // Clear the bulk read cache
        hubSS.clearCache()
        onInitLoop()
    }

    final override fun start() {
        resetRuntime()
        //robot.genTab()
        onStart()
    }

    final override fun loop() {
        // Clear the bulk read cache
        hubSS.clearCache()
        // Draw on Panels
        DrawingUtil.drawPose(robot.follower)

        // Run the Ivy Scheduler to actually update Commands
        Scheduler.execute()

        //Show and update debug
        debugUtil.showAllDebugAuto(robot.follower, alliance, runtime)
        debugUtil.update(telemetry)
        onLoop()
    }

    final override fun stop() {
        StateUtil.endOfAutoPose = robot.follower.pose()
        StateUtil.alliance = alliance
        intakeSS.runIntake().cancel()
        //listOf(firingSS.calcFiring(), firingSS.calcHalf(), firingSS.execFiring()).forEach { it.cancel() }
        onStop()
    }

    // Custom functions
}