package org.firstinspires.ftc.teamcode.config.customOpMode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.geometry.Pose
import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.subSystem.FlowerSS
import org.firstinspires.ftc.teamcode.config.subSystem.IntakeSS
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
    protected lateinit var flowerSS: FlowerSS
    protected var resetPose = Pose(8.0, 8.0, Math.toRadians(90.0))
    protected var rotate = 0.0
    protected var strafe = 0.0
    protected var forward = 0.0


    // Custom lifecycle hooks

    /**
     * Mandatory property that defines which alliance this teleop runs for.
     * Must be overridden by the subclass (e.g. `override val alliance = Alliance.BLUE`)
     */
    abstract val alliance: Robot.Alliance

    /**
     * Mandatory property that defines What kind of OpMode is running
     * Must be overridden by the subclass (e.g. `override val alliance = Alliance.BLUE`)
     */
    abstract val opmode: Robot.OpMode

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
        flowerSS = FlowerSS(robot)
        hubUtil = HubUtil(hardwareMap)
        onInit()
    }

    final override fun init_loop() {
        // Clear the bulk read cache
        hubUtil.clearCache()
        // Draw on Panels
        DrawingUtil.drawOnlyCurrent(robot.follower)
        onInitLoop()
    }

    final override fun start() {
        resetRuntime()
        resetPose = when (alliance) {
            Robot.Alliance.BLUE -> { Pose(8.0, 9.0, Math.toRadians(90.0)) }
            Robot.Alliance.RED -> { Pose(134.0, 9.0, Math.toRadians(90.0)) }
        }
        //robot.genTab()
        robot.follower.activateAllPIDFs()
        onStart()
    }

    final override fun loop() {
        // Clear the bulk read cache
        hubUtil.clearCache()
        // Draw on Panels
        DrawingUtil.drawDebug(robot.follower)
        rotate = gamepad1.right_stick_x.toDouble()
        forward = when (alliance) {
            Robot.Alliance.BLUE -> gamepad1.left_stick_y.toDouble()
            Robot.Alliance.RED -> -gamepad1.left_stick_y.toDouble()
        }
        strafe  = when (alliance) {
            Robot.Alliance.BLUE -> gamepad1.left_stick_x.toDouble()
            Robot.Alliance.RED -> -gamepad1.left_stick_x.toDouble()
        }

        if (gamepad1.rightBumperWasPressed()) { intakeSS.runIntake(robot).also { it.schedule() } }
        else if (gamepad1.rightBumperWasReleased()) { intakeSS.runIntake(robot).cancel() }
        if (gamepad1.leftBumperWasPressed()) { flowerSS.deFlower(robot).schedule() }
        if (gamepad1.crossWasPressed()) robot.follower.pose = resetPose

        // Run the Ivy Scheduler to actually update Commands
        Scheduler.execute()

        //Show and update debug
        debugUtil.showAllDebugTeleop(robot.follower,alliance,runtime,gamepad1,flowerSS,robot)
        debugUtil.update(telemetry)
        onLoop()
    }

    final override fun stop() {
        intakeSS.runIntake(robot).cancel()
        onStop()
    }

    // Custom functions
}