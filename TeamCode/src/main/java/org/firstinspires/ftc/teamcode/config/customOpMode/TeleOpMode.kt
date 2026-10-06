package org.firstinspires.ftc.teamcode.config.customOpMode

import com.pedropathing.drivetrain.DrivePowers
import com.pedropathing.follower.ManualDrive
import com.pedropathing.ivy.Scheduler
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.subSystem.FiringSS
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
    protected lateinit var robot: Robot
    protected lateinit var hubUtil: HubUtil
    protected lateinit var debugUtil: PanelsDebugUtil
    protected lateinit var intakeSS: IntakeSS
    protected lateinit var firingSS: FiringSS
    //protected lateinit var flowerDSS: FlowerDSS
    protected var resetPose = Pose(9.0, 8.8, Math.toRadians(90.0))
    protected var dp: DrivePowers = ManualDrive.fieldCentric(0.0, 0.0, 0.0, 0.0)
    private var flowerDB = 0


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
        robot = Robot(hardwareMap)

        // declare Panels and init the debug util
        robot.initPanels()
        debugUtil = PanelsDebugUtil(robot.panels)
        debugUtil.showInit()
        // Reset Ivy scheduler so commands from a previous OpMode don't carry over
        Scheduler.reset()

        // Init all utils and SS
        robot.initPedro()
        debugUtil.update(telemetry)
        intakeSS = IntakeSS(robot)
        firingSS = FiringSS(robot,robot.follower,alliance)
        //flowerDSS = FlowerDSS(robot)
        hubUtil = HubUtil(hardwareMap)
        firingSS.reset()
        onInit()
    }

    final override fun init_loop() {
        // Clear the bulk read cache
        hubUtil.clearCache()
        // Draw on Panels
        onInitLoop()
    }

    final override fun start() {
        resetRuntime()
        resetPose = when (alliance) {
            Robot.Alliance.BLUE -> { Pose(9.1, 9.2, Math.toRadians(90.0)) }
            Robot.Alliance.RED -> { Pose(132.3, 132.4, Math.toRadians(90.0)) }
        }
        Scheduler.schedule(intakeSS.runIntake())
        Scheduler.schedule(firingSS.calcHalf())
        //robot.genTab()
        onStart()
    }

    final override fun loop() {
        // Clear the bulk read cache
        hubUtil.clearCache()
        // Draw on Panels
        DrawingUtil.drawPose(robot.follower)
        when (alliance) {
            Robot.Alliance.BLUE -> {
                dp = ManualDrive.fieldCentric(
                    gamepad1.left_stick_y.toDouble(),
                    gamepad1.left_stick_x.toDouble(),
                    gamepad1.right_stick_x.toDouble(),
                    robot.follower.pose().heading()
                )
            }
            Robot.Alliance.RED -> {
                dp = ManualDrive.fieldCentric(
                    -gamepad1.left_stick_y.toDouble(),
                    -gamepad1.left_stick_x.toDouble(),
                    gamepad1.right_stick_x.toDouble(),
                    robot.follower.pose().heading()
                )
            }
        }

        if (gamepad1.leftBumperWasPressed()) { intakeSS.reverseIntake(true) }
        if (gamepad1.leftBumperWasReleased()) { intakeSS.reverseIntake(false) }
        if (gamepad1.rightBumperWasPressed()) { firingSS.execFiring().schedule() }
        if (gamepad1.crossWasPressed()) { robot.follower.setPose(resetPose) }
        //if (gamepad1.circleWasReleased()) {
        //    when (flowerDB) {
        //        0 -> {
        //            flowerDSS.go()
        //            flowerDB = 1
        //            return
        //        }
        //        1 -> {
        //            flowerDSS.back()
        //            flowerDB = 0
        //            return
        //        }
        //        else -> throw IllegalStateException("WTF How did you even mess up this much????")
        //    }
        //}

        // Run the Ivy Scheduler to actually update Commands
        Scheduler.execute()

        //Show and update debug
        debugUtil.showAllDebugTeleop(robot.follower,alliance,runtime,gamepad1,firingSS.getHalf(),robot)
        debugUtil.update(telemetry)
        onLoop()
    }

    final override fun stop() {
        intakeSS.runIntake().cancel()
        //listOf(firingSS.calcFiring(), firingSS.calcHalf(), firingSS.execFiring()).forEach { it.cancel() }
        onStop()
    }

    // Custom functions
}