package org.firstinspires.ftc.teamcode.config.customOpMode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.subSystem.IntakeSS
import org.firstinspires.ftc.teamcode.config.subSystem.LLSS
import org.firstinspires.ftc.teamcode.config.util.DrawingUtil
import org.firstinspires.ftc.teamcode.config.util.HubUtil
import org.firstinspires.ftc.teamcode.config.util.PanelsDebugUtil

/**
 * Custom-made OpMode for making a TeleOP that has needed code for a OpMode
 * @author Jonny Todd - 29403 PiBytes
 */
abstract class TeleOpMode : OpMode() {

    // Shared resources
    private var panels: TelemetryManager? = null
    protected lateinit var robot: Robot
    protected lateinit var hubUtil: HubUtil
    protected lateinit var debugUtil: PanelsDebugUtil
    protected lateinit var intakeSS: IntakeSS
    protected lateinit var llss: LLSS
    protected lateinit var follower: Follower
    protected var resetPose = Pose(8.0, 8.0, Math.toRadians(90.0))
    protected var rotate = 0.0
    protected var strafe = 0.0
    protected var forward = 0.0
    protected val autoTurnPixel = 2.0
    protected val autoTurnRad = Math.toRadians(1.0)
    protected val autoTurnTimeoutSec = 0.5
    open var autoTurnStartTime = 0.0
    open var isAutoTurning = false


    // Custom lifecycle hooks

    /**
     * Mandatory property that defines which alliance this teleop runs for.
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

        // Init all utils and SS
        robot = Robot(hardwareMap)
        debugUtil.update(telemetry)
        intakeSS = IntakeSS(robot)
        llss = LLSS(robot)
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
            Robot.Alliance.BLUE -> { Pose(8.0, 8.0, Math.toRadians(90.0)) }
            Robot.Alliance.RED -> { Pose(134.0, 7.0, Math.toRadians(90.0)) }
        }
        follower.activateAllPIDFs()
        onStart()
    }

    final override fun loop() {
        // Clear the bulk read cache
        hubUtil.clearCache()
        // Draw on Panels
        if (::follower.isInitialized) { DrawingUtil.drawDebug(follower) }
        rotate = gamepad1.right_stick_x.toDouble()
        forward = when (alliance) {
            Robot.Alliance.BLUE -> gamepad1.left_stick_y.toDouble()
            Robot.Alliance.RED -> -gamepad1.left_stick_y.toDouble()
        }
        strafe  = when (alliance) {
            Robot.Alliance.BLUE -> gamepad1.left_stick_x.toDouble()
            Robot.Alliance.RED -> -gamepad1.left_stick_x.toDouble()
        }

        if (gamepad1.rightBumperWasPressed()) { intakeSS.runIntakeCommand.schedule() }
        if (gamepad1.rightBumperWasReleased()) { intakeSS.runIntakeCommand.cancel() }
        if (gamepad1.crossWasReleased()) follower.pose = resetPose

        // Run the Ivy Scheduler to actually update Commands
        Scheduler.execute()

        //Show and update debug
        debugUtil.showAllDebugTeleop(follower,alliance,runtime,gamepad1,llss,autoTurnPixel, robot)
        debugUtil.update(telemetry)
        onLoop()
    }

    final override fun stop() {
        llss.stop(robot)
        if (intakeSS.runIntakeCommand.isScheduled) intakeSS.runIntakeCommand.cancel()
        onStop()
    }

    // Custom functions
}