package org.firstinspires.ftc.teamcode.config.customOpMode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.config.subSystem.IntakeSS
import org.firstinspires.ftc.teamcode.config.subSystem.LLSS
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
    protected lateinit var hubUtil : HubUtil
    protected lateinit var debugUtil : PanelsDebugUtil
    protected lateinit var intakeSS : IntakeSS
    protected lateinit var llss: LLSS
    protected lateinit var follower : Follower
    protected var resetPose = Pose(8.0, 8.0, Math.toRadians(90.0))
    protected var rotate = 0.0
    protected var strafe = 0.0
    protected var forward = 0.0
    protected val autoTurnPixel = 1.0
    protected val autoTurnTimeoutSec = 0.5
    open var autoTurnStartTime = 0.0
    open var isAutoTurning = false


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

        // Init all utils and SS
        debugUtil.update(telemetry)
        intakeSS = IntakeSS(hardwareMap)
        llss = LLSS(hardwareMap)
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
                Pose(134.0, 7.0, Math.toRadians(90.0))
            }
        }
        onStart()
    }

    final override fun loop() {
        // Clear the bulk read cache
        hubUtil.clearCache()
        // Draw on Panels
        if (::follower.isInitialized) {
            DrawingUtil.drawDebug(follower)
        }

        rotate = gamepad1.right_stick_x.toDouble()
        forward = when (alliance) {
            Alliance.BLUE -> gamepad1.left_stick_y.toDouble()
            Alliance.RED -> -gamepad1.left_stick_y.toDouble()
        }
        strafe  = when (alliance) {
            Alliance.BLUE -> gamepad1.left_stick_x.toDouble()
            Alliance.RED -> -gamepad1.left_stick_x.toDouble()
        }

        if (gamepad1.left_bumper) intakeSS.runIntakeCommand.start()
        else intakeSS.runIntakeCommand.cancel()
        if (gamepad1.cross) follower.pose = resetPose

        //Show and update debug
        debugUtil.showAllDebugTeleop(follower, hubUtil, alliance, runtime, gamepad1, llss, autoTurnPixel)
        debugUtil.update(telemetry)
        onLoop()
    }

    final override fun stop() {
        llss.stop()
        if (intakeSS.runIntakeCommand.isScheduled) intakeSS.runIntakeCommand.cancel()
        onStop()
    }

    // Custom functions
}
