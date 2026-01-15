package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Servo
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.util.DriveUtil
import org.firstinspires.ftc.teamcode.util.EndgameUtil
import org.firstinspires.ftc.teamcode.util.IntakeUtil
import org.firstinspires.ftc.teamcode.util.LimelightUtil
import org.firstinspires.ftc.teamcode.util.LoaderTracker
import org.firstinspires.ftc.teamcode.util.OuttakeController
import org.firstinspires.ftc.teamcode.util.TeleOpConfig

@TeleOp(name = "Red TeleOP (UTIL REFACTOR)", group = "Main Red")
class NewRedTeleOpUtilRefactor : OpMode() {

    private var panels: TelemetryManager? = null
    private val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())

    private var jobDetections: Job? = null
    private var jobOuttake: Job? = null
    private var jobIntake: Job? = null
    private var jobFiring: Job? = null

    private val startPose = Pose(72.0, 72.0, Math.toRadians(0.0))
    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    private lateinit var outTake1: DcMotorEx
    private lateinit var outTake2: DcMotorEx
    private lateinit var liftLeft: DcMotorEx
    private lateinit var liftRight: DcMotorEx
    private lateinit var intakeServo1: CRServo

    private lateinit var bowlServo: Servo
    private lateinit var camServo: Servo
    private lateinit var limelight: Limelight3A

    private lateinit var colorSensor: ColorSensor
    private lateinit var statusLEDR: DigitalChannel
    private lateinit var statusLEDG: DigitalChannel

    // State flags
    @Volatile private var isDispensing = false
    private var rightBumperPressed = false
    private var dPadDownPressed = false

    // PIDF config you already had
    private val pidP = 150.0
    private val pidI = 0.0
    private val pidD = 0.0
    private val pidF = 13.5

    private lateinit var loader: LoaderTracker
    private lateinit var outtake: OuttakeController

    override fun init() {
        initializeHardware()
        initializePedroPathing()

        loader = LoaderTracker(initialOrd = arrayOf("P", "G", "P")) // keep your current default
        outtake = OuttakeController(outTake1, outTake2, LimelightUtil.Tags.RED_DEPO)

        // Limelight helper
        LimelightUtil.init(limelight, bowlServo, camServo)
        LimelightUtil.setPipeline(0)
    }

    override fun start() {
        resetServos()
        opmodeTimer.resetTimer()
        follower.startTeleOpDrive()

        jobOuttake = scope.launch {
            while (isActive) {
                outtake.update()
                delay(5)
            }
        }

        jobIntake = scope.launch {
            while (isActive) {
                val isFull = if (gamepad1.circle) true else loader.isFull()
                IntakeUtil.apply(isFull, intakeServo1, statusLEDR, statusLEDG)
                delay(5)
            }
        }

        jobDetections = scope.launch {
            while (isActive) {
                if (!isDispensing) {
                    val res = loader.updateFromSensor(
                        g = colorSensor.green(),
                        b = colorSensor.blue(),
                        nowMs = System.currentTimeMillis()
                    )
                    if (res != null) {
                        bowlServo.position = res.newBowlPosition
                    }
                }
                delay(25)
            }
        }

        jobFiring = scope.launch {
            while (isActive) {
                handleFiring()
                delay(25)
            }
        }
    }

    override fun loop() {
        follower.update()

        // ---- DRIVE ----
        val slowMode = gamepad1.left_bumper
        val cmd = DriveUtil.commandFrom(
            gamepad = gamepad1,
            isDispensing = isDispensing,
            slowMode = slowMode,
            normalCap = TeleOpConfig.DriveCaps.NORMAL_MAX,
            slowCap = TeleOpConfig.DriveCaps.SLOW_MAX,
            fieldCentric = true
        )

        follower.setMaxPower(cmd.maxPower)
        follower.setTeleOpDrive(cmd.forward, cmd.strafe, cmd.rotate, cmd.fieldCentric)

        // ---- TELEMETRY ----
        panels?.debug("OutTake 1 Power", outTake1.power)
        panels?.debug("OutTake 2 Power", outTake2.power)
        panels?.debug("OutTake 1 velocity", outTake1.velocity)
        panels?.debug("OutTake 2 velocity", outTake2.velocity)
        panels?.debug("OUTTAKE_POWER", outtake.lastPower)

        val ord = loader.snapshotOrd()
        panels?.debug("ord[0]", ord[0])
        panels?.debug("ord[1]", ord[1])
        panels?.debug("ord[2]", ord[2])

        panels?.debug("Drive maxPower", cmd.maxPower)
        panels?.debug("Run Time", runtime)
        panels?.update(telemetry)

        // ---- Manual bowl nudge (kept) ----
        if (gamepad1.dpad_down && !dPadDownPressed) {
            bowlServo.position -= 0.1
        } else if (!gamepad1.dpad_down && dPadDownPressed) {
            bowlServo.position += 0.1
        }
        dPadDownPressed = gamepad1.dpad_down

        // ---- Endgame kill / lift mode (kept) ----
        if (gamepad1.dpad_up && gamepad1.triangle) {
            EndgameUtil.applyLiftLogic(liftLeft, liftRight)

            jobOuttake?.cancel()
            outtake.stop()

            jobIntake?.cancel()
            intakeServo1.power = 0.0
        }
    }

    override fun stop() {
        jobOuttake?.cancel()
        jobIntake?.cancel()
        jobDetections?.cancel()
        jobFiring?.cancel()
        outtake.stop()
        intakeServo1.power = 0.0
        LimelightUtil.stop()
    }

    // -------------------------
    // Center + Fire on RIGHT bumper using LimelightUtil
    // -------------------------
    private suspend fun handleFiring() {
        val centerPressed = gamepad1.right_bumper

        if (centerPressed && !rightBumperPressed && !isDispensing) {
            isDispensing = true
            try {
                follower.setMaxPower(TeleOpConfig.DriveCaps.DISPENSE_MAX)

                // Center on RED_DEPO and fire loaded balls based on ord[]
                LimelightUtil.centerAndFire(
                    tagId = LimelightUtil.Tags.RED_DEPO,
                    follower = follower,
                    slots = loader.snapshotOrd()
                )

                // reset after firing
                loader.clearOrd("N")
                bowlServo.position = TeleOpConfig.ServoPositions.LOAD_P1
            } finally {
                isDispensing = false
            }
        }

        rightBumperPressed = centerPressed
    }

    // -------------------------
    // Hardware + setup
    // -------------------------
    private fun initializeHardware() {
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        liftLeft = hardwareMap.get(DcMotorEx::class.java, "liftLeft")
        liftRight = hardwareMap.get(DcMotorEx::class.java, "liftRight")
        intakeServo1 = hardwareMap.get(CRServo::class.java, "intakeServo1")
        bowlServo = hardwareMap.get(Servo::class.java, "bowlServo")
        camServo = hardwareMap.get(Servo::class.java, "camServo")
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        colorSensor = hardwareMap.get(ColorSensor::class.java, "ColorSensor")
        statusLEDR = hardwareMap.get(DigitalChannel::class.java, "statusLEDR")
        statusLEDG = hardwareMap.get(DigitalChannel::class.java, "statusLEDG")

        statusLEDR.mode = DigitalChannel.Mode.OUTPUT
        statusLEDG.mode = DigitalChannel.Mode.OUTPUT

        setupMotorDirections()
        setupPIDFCoefficients()
        resetEncoders()
        setupVision()
    }

    private fun setupVision() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
        limelight.start()
    }

    private fun setupMotorDirections() {
        listOf(outTake2, liftLeft).forEach { it.direction = DcMotorSimple.Direction.REVERSE }
    }

    private fun setupPIDFCoefficients() {
        listOf(outTake1, outTake2).forEach { it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF) }
    }

    private fun resetEncoders() {
        listOf(outTake1, outTake2).forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        listOf(liftLeft, liftRight).forEach { motor ->
            motor.targetPosition = TeleOpConfig.EndGame.LIFTMAX
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    private fun resetServos() {
        camServo.position = TeleOpConfig.ServoPositions.CAM_CLOSED
        bowlServo.position = TeleOpConfig.ServoPositions.FIRE_P2
    }

    private fun initializePedroPathing() {
        panels = PanelsTelemetry.telemetry

        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer().apply { resetTimer() }

        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()
    }
}