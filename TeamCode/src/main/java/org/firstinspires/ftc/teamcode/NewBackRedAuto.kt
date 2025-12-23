package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

@Autonomous(name = "Back Red Auto (NEW)", group = "Main Red")
class NewBackRedAuto : OpMode() {
    var panels: TelemetryManager? = null
    private val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    private var colorDetection: Job? = null
    private var outTakeCalc: Job? = null

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

    private var pathState: Int = 0
    private var timerState = false
    private var intake = 0
    private var isSeen = false
    @Volatile
    private var isDispensing = false

    // Piece tracking array: "N" = None, "G" = Green, "P" = Purple
    var ord = arrayOf("N", "N", "N")

    // Poses - same strategy as before
    private val startPose = Pose(72.0, 0.0, Math.toRadians(90.0))
    private val preloadPose = Pose(74.0, 4.0, Math.toRadians(80.0))
    private val pickupPoint5 = Pose(80.0, 17.5, Math.toRadians(9.0))
    private val pickup1 = Pose(86.5, 24.0, Math.toRadians(0.0))
    private val pickup1Ball1 = Pose(93.3, 24.0, Math.toRadians(0.0))
    private val pickup1Ball2 = Pose(96.3, 24.0, Math.toRadians(0.0))
    private val pickup1Ball3 = Pose(104.8, 24.0, Math.toRadians(0.0))
    private val scoreBack = Pose(74.0, 10.0, Math.toRadians(73.0))

    // Path chains
    private lateinit var preloadPath: PathChain
    private lateinit var pickupPointPath: PathChain
    private lateinit var pickupPath1: PathChain
    private lateinit var pickupBall1Path: PathChain
    private lateinit var pickupBall2Path: PathChain
    private lateinit var pickupBall3Path: PathChain
    private lateinit var returnPath: PathChain

    // PID coefficients matching new TeleOP
    private val pidP = 150.0
    private val pidI = 0.0
    private val pidD = 0.0
    private val pidF = 13.5
    private var velocityModeInitialized = false
    private var velocityPowerScale = 1.0

    object ServoPositions {
        // Loading positions
        const val LOAD_P1 = 0.004
        const val LOAD_P2 = 0.080
        const val LOAD_P3 = 0.153

        // Firing/dispensing positions
        const val FIRE_P1 = 0.118
        const val FIRE_P2 = 0.1885
        const val FIRE_P3 = 0.042

        // Camera servo positions
        const val CAM_OPEN = 0.5
        const val CAM_CLOSED = 0.255

        const val INTAKE_ON = 1.0
        const val INTAKE_REVERSE = -1.0
        const val INTAKE_OFF = 0.0
    }

    object AprilTagIds {
        const val RED_DEPO = 24
    }

    object DepoCenter {
        const val CAM_WIDTH_PX = 1280
        const val CENTER_DEADZONE = 15
        const val KP_ROTATE = 0.003
        var OUTTAKE_SPEED = 0.20
    }

    object Timing {
        const val DISPENSE_INITIAL_DELAY = 100L
        const val BOWL_MOVE_DELAY = 650L
        const val CAM_OPEN_DELAY = 400L
        const val CAM_CLOSE_DELAY = 900L
        const val DETECTION_COOLDOWN = 1500L
    }

    override fun init() {
        initializeHardware()
        initializePedroPathing()
        buildPaths()
    }

    override fun start() {
        limelight.start()
        opmodeTimer.resetTimer()
        setPathState(0)

        // Start outtake power calculation coroutine
        outTakeCalc = scope.launch {
            while (isActive) {
                outTakePower()
                delay(5)
            }
        }

        // Start color detection coroutine
        colorDetection = scope.launch {
            while (isActive) {
                handleDetections()
                delay(25)
            }
        }
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()
        handleIntake()

        // Debug telemetry
        panels?.debug("Path State", pathState)
        panels?.debug("Order", "${ord[0]}${ord[1]}${ord[2]}")
        panels?.debug("Timer", pathTimer.elapsedTimeSeconds)
        panels?.debug("Is Dispensing", isDispensing)
        panels?.update(telemetry)
    }

    override fun stop() {
        limelight.stop()
        colorDetection?.cancel()
        outTakeCalc?.cancel()
    }

    private fun buildPaths() {
        preloadPath = follower.pathBuilder()
            .addPath(BezierCurve(startPose, preloadPose))
            .setLinearHeadingInterpolation(startPose.heading, preloadPose.heading)
            .build()

        pickupPointPath = follower.pathBuilder()
            .addPath(BezierCurve(preloadPose, pickupPoint5))
            .setLinearHeadingInterpolation(preloadPose.heading, pickupPoint5.heading)
            .build()

        pickupPath1 = follower.pathBuilder()
            .addPath(BezierCurve(pickupPoint5, pickup1))
            .setLinearHeadingInterpolation(pickupPoint5.heading, pickup1.heading)
            .build()

        pickupBall1Path = follower.pathBuilder()
            .addPath(BezierCurve(pickup1, pickup1Ball1))
            .setLinearHeadingInterpolation(pickup1.heading, pickup1Ball1.heading)
            .build()

        pickupBall2Path = follower.pathBuilder()
            .addPath(BezierCurve(pickup1Ball1, pickup1Ball2))
            .setLinearHeadingInterpolation(pickup1Ball1.heading, pickup1Ball2.heading)
            .build()

        pickupBall3Path = follower.pathBuilder()
            .addPath(BezierCurve(pickup1Ball2, pickup1Ball3))
            .setLinearHeadingInterpolation(pickup1Ball2.heading, pickup1Ball3.heading)
            .build()

        returnPath = follower.pathBuilder()
            .addPath(BezierCurve(pickup1Ball3, scoreBack))
            .setLinearHeadingInterpolation(pickup1Ball3.heading, scoreBack.heading)
            .build()
    }

    private fun autonomousPathUpdate() {
        val notBusy = !follower.isBusy

        when (pathState) {
            // Initial state - start preload path
            0 -> {
                if (notBusy && !timerState) {
                    pathTimer.resetTimer()
                    timerState = true
                    follower.followPath(preloadPath, true)
                    setPathState(1)
                }
            }

            // Wait for preload path to complete
            1 -> {
                if (notBusy) {
                    setPathState(2)
                }
            }

            // Small delay before centering
            2 -> {
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    setPathState(3)
                }
            }

            // Center on depo and shoot preload
            3 -> {
                centerDepo()
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    setPathState(4)
                }
            }

            // Execute preload dispense
            4 -> {
                if (!isDispensing) {
                    isDispensing = true
                    scope.launch {
                        executeDispensing()
                        isDispensing = false
                        setPathState(5)
                    }
                }
            }

            // Start heading to pickup zone
            5 -> {
                if (!isDispensing && notBusy) {
                    follower.followPath(pickupPointPath, false)
                    setPathState(6)
                }
            }

            // Wait to reach pickup point, then continue
            6 -> {
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.5) {
                        follower.setMaxPower(0.4)
                        follower.followPath(pickupPath1, true)
                        setPathState(7)
                    }
                }
            }

            // Start slow pickup - ball 1
            7 -> {
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.2) {
                        follower.setMaxPower(0.18)
                        intake = 1
                        follower.followPath(pickupBall1Path, true)
                        setPathState(8)
                    }
                }
            }

            // Continue to ball 2
            8 -> {
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.2) {
                        follower.setMaxPower(0.2)
                        follower.followPath(pickupBall2Path, true)
                        setPathState(9)
                    }
                }
            }

            // Continue to ball 3
            9 -> {
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 1.0) {
                        follower.setMaxPower(0.2)
                        follower.followPath(pickupBall3Path, true)
                        setPathState(10)
                    }
                }
            }

            // Check if we have all 3 pieces, then return to score
            10 -> {
                val isFull = ord.none { it == "N" }
                if (notBusy || isFull) {
                    intake = 0
                    follower.setMaxPower(0.6)
                    follower.followPath(returnPath, true)
                    setPathState(11)
                }
            }

            // Wait to reach scoring position
            11 -> {
                if (notBusy) {
                    setPathState(12)
                }
            }

            // Small delay before centering
            12 -> {
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    setPathState(13)
                }
            }

            // Center on depo for final score
            13 -> {
                intake = 0
                centerDepo()
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    setPathState(14)
                }
            }

            // Execute final dispense
            14 -> {
                if (!isDispensing) {
                    isDispensing = true
                    scope.launch {
                        executeDispensing()
                        isDispensing = false
                        setPathState(15)
                    }
                }
            }

            // Done!
            15 -> {
                // Autonomous complete
            }
        }
    }

    private fun handleIntake() {
        val isFull = ord.none { it == "N" }
        when (intake) {
            0 -> {
                intakeServo1.power = ServoPositions.INTAKE_OFF
            }
            1 -> {
                if (isFull) {
                    intakeServo1.power = ServoPositions.INTAKE_REVERSE
                } else {
                    intakeServo1.power = ServoPositions.INTAKE_ON
                }
            }
        }
    }

    private fun handleDetections() {
        if (isDispensing) return

        val r = colorSensor.red()
        val g = colorSensor.green()
        val b = colorSensor.blue()

        // Reset seen flag when no piece detected
        if (g <= 80 && b <= 110) {
            isSeen = false
        }

        // Purple detection
        if (g >= 80 && b >= 110 && g < 110) {
            if (!isSeen) {
                val slot = nextSlot()
                if (slot != -1) {
                    ord[slot] = "P"
                    advanceBowl(slot)
                    Thread.sleep(Timing.DETECTION_COOLDOWN)
                }
                isSeen = true
            }
        }

        // Green detection
        if (g >= 110) {
            if (!isSeen) {
                val slot = nextSlot()
                if (slot != -1) {
                    ord[slot] = "G"
                    advanceBowl(slot)
                    Thread.sleep(Timing.DETECTION_COOLDOWN)
                }
                isSeen = true
            }
        }
    }

    private fun nextSlot(): Int {
        return when {
            ord[0] == "N" -> 0
            ord[1] == "N" -> 1
            ord[2] == "N" -> 2
            else -> -1
        }
    }

    private fun advanceBowl(slot: Int) {
        bowlServo.position = when (slot) {
            0 -> ServoPositions.LOAD_P2
            1 -> ServoPositions.LOAD_P3
            2 -> ServoPositions.FIRE_P2
            else -> bowlServo.position
        }
    }

    private suspend fun executeDispensing() {
        val isFull = ord.none { it == "N" }

        if (isFull) {
            // Fixed dispense sequence - no sorting needed
            val dispenseSequence = listOf(
                ServoPositions.FIRE_P2,
                ServoPositions.FIRE_P1,
                ServoPositions.FIRE_P3
            )
            executeDispenseSequence(dispenseSequence)
        }

        // Reset
        delay(100)
        bowlServo.position = ServoPositions.LOAD_P1
        delay(100)

        ord = arrayOf("N", "N", "N")
    }

    private suspend fun executeDispenseSequence(positions: List<Double>) {
        delay(Timing.DISPENSE_INITIAL_DELAY)

        positions.forEach { position ->
            bowlServo.position = position
            delay(Timing.BOWL_MOVE_DELAY)

            camServo.position = ServoPositions.CAM_OPEN
            delay(Timing.CAM_OPEN_DELAY)

            camServo.position = ServoPositions.CAM_CLOSED
            delay(Timing.CAM_CLOSE_DELAY)
        }
    }

    private fun centerDepo() {
        val result: LLResult? = limelight.latestResult
        if (result == null || !result.isValid) {
            return
        }

        val fiducialResults = result.fiducialResults
        val target = fiducialResults.firstOrNull { it.fiducialId == AprilTagIds.RED_DEPO }

        if (target == null) {
            return
        }

        val xErrPx: Double = target.targetXPixels - (DepoCenter.CAM_WIDTH_PX / 2.0)

        if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
            // Centered - advance to next state
            if (pathState == 3) {
                setPathState(4)
            } else if (pathState == 13) {
                setPathState(14)
            }
            return
        }

        // Calculate turn correction
        val hFovDeg = 70.0
        val hFovRad = Math.toRadians(hFovDeg)
        val pixelsToRad = hFovRad / DepoCenter.CAM_WIDTH_PX
        val angleError = xErrPx * pixelsToRad
        val maxTurnStepRad = Math.toRadians(10.0)
        val turnStep = clip(angleError, -maxTurnStepRad, maxTurnStepRad)

        val currentHeading = follower.pose.heading
        val targetHeading = currentHeading - turnStep
        follower.turnTo(targetHeading)
    }

    private fun outTakePower() {
        val result: LLResult? = limelight.latestResult
        if (result == null || !result.isValid) {
            return
        }

        val fiducialResults = result.fiducialResults
        val target = fiducialResults.firstOrNull { it.fiducialId == AprilTagIds.RED_DEPO }

        if (target == null) {
            return
        }

        val targetY = target.targetYPixels
        val powerResult = 0.000204 * targetY + 0.15
        DepoCenter.OUTTAKE_SPEED = powerResult

        outTake1.power = DepoCenter.OUTTAKE_SPEED
        outTake2.power = DepoCenter.OUTTAKE_SPEED
    }

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

        camServo.position = ServoPositions.CAM_CLOSED
        bowlServo.position = ServoPositions.LOAD_P1

        setupMotorDirections()
        setupPIDFCoefficients()
        resetEncoders()
        setupVision()
    }

    private fun setupVision() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
    }

    private fun setupMotorDirections() {
        listOf(outTake2, liftLeft)
            .forEach { it.direction = DcMotorSimple.Direction.REVERSE }
    }

    private fun setupPIDFCoefficients() {
        listOf(outTake1, outTake2)
            .forEach { it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF) }
    }

    private fun resetEncoders() {
        listOf(outTake1, outTake2).forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        listOf(liftLeft, liftRight).forEach { motor ->
            motor.targetPosition = 11400 // EndGame.LIFTMAX
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    private fun initializePedroPathing() {
        panels = PanelsTelemetry.telemetry

        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()

        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()
    }

    private fun ensureVelocityMode() {
        if (!velocityModeInitialized) {
            outTake1.mode = DcMotor.RunMode.RUN_USING_ENCODER
            outTake2.mode = DcMotor.RunMode.RUN_USING_ENCODER
            velocityModeInitialized = true
        }
    }

    private fun powerToTicksPerSecond(motor: DcMotorEx, power: Double): Double {
        val clipped = max(-1.0, min(1.0, power))
        val maxRpm = motor.motorType.maxRPM
        val tpr = motor.motorType.ticksPerRev
        val maxTicksPerSec = (maxRpm * tpr) / 60.0
        return clipped * velocityPowerScale * maxTicksPerSec
    }

    private fun setMotorVelocityFromPseudoPower(motor: DcMotorEx, power: Double) {
        ensureVelocityMode()
        motor.velocity = powerToTicksPerSecond(motor, power)
    }

    private fun clip(v: Double, min: Double, max: Double): Double {
        return max(min, min(max, v))
    }

    private fun setPathState(pState: Int) {
        pathState = pState
        pathTimer.resetTimer()
        timerState = false
    }
}
