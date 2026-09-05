package org.firstinspires.ftc.teamcode.config.customOpMode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.subSystem.IntakeSS
import org.firstinspires.ftc.teamcode.config.util.DrawingUtil
import org.firstinspires.ftc.teamcode.config.util.HubUtil
import org.firstinspires.ftc.teamcode.config.util.PanelsDebugUtil
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

/**
 * Custom-made OpMode for making an Auto that has needed code for a OpMode
 * @author Jonny Todd - 29403 PiBytes
 */
abstract class AutoOpMode : OpMode() {

    // Shared resources
    private var panels: TelemetryManager? = null
    protected lateinit var robot: Robot
    protected lateinit var hubUtil: HubUtil
    protected lateinit var debugUtil: PanelsDebugUtil
    protected lateinit var intakeSS: IntakeSS
    protected lateinit var follower: Follower

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
        robot = Robot(hardwareMap)
        intakeSS = IntakeSS(robot)
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
        onStart()
    }

    final override fun loop() {
        // Clear the bulk read cache
        hubUtil.clearCache()
        // Draw on Panels
        if (::follower.isInitialized) { DrawingUtil.drawDebug(follower) }

        // Run the Ivy Scheduler to actually update Commands
        Scheduler.execute()

        //Show and update debug
        debugUtil.showAllDebugAuto(follower, hubUtil, alliance, runtime)
        debugUtil.update(telemetry)
        onLoop()
    }

    final override fun stop() {
        if (::follower.isInitialized) { VariableStateUtil.endOfAutoPose = follower.pose }
        VariableStateUtil.alliance = alliance
        if (intakeSS.runIntakeCommand.isScheduled) intakeSS.runIntakeCommand.cancel()
        onStop()
    }

    // Custom functions
    fun runAuto(): Command {
        // Example for PedroPathing branch in Kotlin
        //var cases = LinkedHashMap<BooleanSupplier, Command>()
        //cases[BooleanSupplier {follower.distanceRemaining <= 15.0}] = rampSS.rampIntake()
        //val handlePos: Command = Commands.branch(cases)
        return Groups.sequential(
            follow(follower, Robot.AutoPoseUtil.startToLeftCorner,true, 0.5),
            Commands.waitMs(750.0),
            follow(follower,Robot.AutoPoseUtil.bottomLeftCornerToLeftSpike,true, 0.5),
            follow(follower, Robot.AutoPoseUtil.leftSpikeToHiveFour,true,0.5),
        )
    }
}