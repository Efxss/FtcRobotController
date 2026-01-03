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
import com.qualcomm.robotcore.eventloop.opmode.Disabled
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

@Disabled
class NewOldBackRedAuto : OpMode() {
    var panels: TelemetryManager? = null
    val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    var colorDetection: Job? = null
    var outTakeCalc: Job? = null
    var pathUpdate: Job? = null
    var doIntake: Job? = null

    lateinit var follower: Follower
    lateinit var pathTimer: Timer
    lateinit var actionTimer: Timer
    lateinit var opmodeTimer: Timer

    lateinit var outTake1: DcMotorEx
    lateinit var outTake2: DcMotorEx
    lateinit var liftLeft: DcMotorEx
    lateinit var liftRight: DcMotorEx
    lateinit var intakeServo1: CRServo
    lateinit var bowlServo: Servo
    lateinit var camServo: Servo
    lateinit var limelight: Limelight3A
    lateinit var colorSensor: ColorSensor

    @set:JvmName("PathStateVar")
    var pathState: Int = 0
    var timerState = false
    var intake = 0
    var isSeen = false
    @Volatile var isDispensing = false
    @Volatile var isCentering = false
    @Volatile var centeringStarted = false
    @Volatile var centeringComplete = false

    // Piece tracking array: "N" = None, "G" = Green, "P" = Purple
    var ord = arrayOf("N", "N", "N")

    // Poses - same strategy as before
    val startPose = Pose(72.0, 0.0, Math.toRadians(90.0))
    val preloadPose = Pose(74.0, 4.0, Math.toRadians(80.0))
    val pickupPoint5 = Pose(80.0, 17.5, Math.toRadians(9.0))
    val pickup1 = Pose(86.5, 24.0, Math.toRadians(0.0))
    val pickup1Ball1 = Pose(93.3, 24.0, Math.toRadians(0.0))
    val pickup1Ball2 = Pose(96.3, 24.0, Math.toRadians(0.0))
    val pickup1Ball3 = Pose(104.8, 24.0, Math.toRadians(0.0))
    val scoreBack = Pose(74.0, 10.0, Math.toRadians(73.0))

    // Path chains
    lateinit var preloadPath: PathChain
    lateinit var pickupPointPath: PathChain
    lateinit var pickupPath1: PathChain
    lateinit var pickupBall1Path: PathChain
    lateinit var pickupBall2Path: PathChain
    lateinit var pickupBall3Path: PathChain
    lateinit var returnPath: PathChain

    // PID coefficients matching new TeleOP
    val pidP = 150.0
    val pidI = 0.0
    val pidD = 0.0
    val pidF = 13.5
    var velocityModeInitialized = false
    var velocityPowerScale = 1.0

    // Centering timeout and retry limits
    val centeringTimeoutSeconds = 3.0
    val maxCenteringAttempts = 50

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
        const val CAM_OPEN = 0.43
        const val CAM_CLOSED = 0.255

        const val INTAKE_ON = 1.0
        const val INTAKE_REVERSE = -1.0
        const val INTAKE_OFF = 0.0
    }

    object AprilTagIds {
        const val GPP_ORDER = 21
        const val PGP_ORDER = 22
        const val PPG_ORDER = 23
        const val RED_DEPO = 24
    }

    object DepoCenter {
        const val DESIRED_TAG_WIDTH_PX = 80
        const val ROTATE_POWER = 0.2
        const val CAM_WIDTH_PX = 1280
        const val CAM_HEIGHT_PX = 960
        const val CENTER_DEADZONE = 15
        const val KP_ROTATE = 0.003
        var OUTTAKE_SPEED = 0.20
    }

    object Timing {
        const val DISPENSE_INITIAL_DELAY = 100L
        const val BOWL_MOVE_DELAY = 500L
        const val CAM_OPEN_DELAY = 200L
        const val CAM_CLOSE_DELAY = 350L
        const val DETECTION_COOLDOWN = 1500L
        const val OUTTAKE_DELAY = 800L
        const val CENTERING_POLL_DELAY = 50L
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
                try {
                    outTakePower()
                } catch (e: Exception) {
                    panels?.debug("OutTake Error", e.message ?: "Unknown")
                }
                delay(5)
            }
        }

        // Start color detection coroutine
        colorDetection = scope.launch {
            while (isActive) {
                try {
                    handleDetections()
                } catch (e: Exception) {
                    panels?.debug("Detection Error", e.message ?: "Unknown")
                }
                delay(25)
            }
        }

        doIntake = scope.launch {
            while (isActive) {
                try {
                    handleIntake()
                } catch (e: Exception) {
                    panels?.debug("Intake Error", e.message ?: "Unknown")
                }
                delay(10)
            }
        }

        pathUpdate = scope.launch {
            while (isActive) {
                try {
                    follower.update()
                    autonomousPathUpdate()
                } catch (e: Exception) {
                    panels?.debug("Path Error", e.message ?: "Unknown")
                }
                delay(10) // Increased from 1ms to 10ms for stability
            }
        }
    }

    override fun loop() {
        // Debug telemetry
        panels?.debug("Path State", pathState)
        panels?.debug("Order", "${ord[0]}${ord[1]}${ord[2]}")
        panels?.debug("Timer", pathTimer.elapsedTimeSeconds)
        panels?.debug("Is Dispensing", isDispensing)
        panels?.debug("Is Centering", isCentering)
        panels?.debug("Centering Complete", centeringComplete)
        panels?.update(telemetry)
    }

    override fun stop() {
        limelight.stop()
        colorDetection?.cancel()
        outTakeCalc?.cancel()
        pathUpdate?.cancel()
        doIntake?.cancel()
    }

    fun buildPaths() {
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

    suspend fun autonomousPathUpdate() {
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

            // Small delay before centering - properly reset timer
            2 -> {
                if (!timerState) {
                    pathTimer.resetTimer()
                    timerState = true
                }
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    // Reset centering flags before starting
                    centeringStarted = false
                    centeringComplete = false
                    isCentering = false
                    setPathState(3)
                }
            }

            // Center on depo and shoot preload
            3 -> {
                if (!centeringStarted) {
                    centeringStarted = true
                    pathTimer.resetTimer()
                }

                // Only attempt centering if follower is not busy
                if (notBusy && !isCentering) {
                    val centered = centerDepo()
                    if (centered) {
                        centeringComplete = true
                        setPathState(4)
                    }
                }

                // Timeout fallback
                if (pathTimer.elapsedTimeSeconds > centeringTimeoutSeconds) {
                    panels?.debug("Centering", "Timeout - proceeding anyway")
                    centeringComplete = true
                    setPathState(4)
                }
            }

            // Execute preload dispense
            4 -> {
                if (!isDispensing) {
                    isDispensing = true
                    try {
                        executeDispensing()
                    } catch (e: Exception) {
                        panels?.debug("Dispense Error", e.message ?: "Unknown")
                    } finally {
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

            // Small delay before centering - properly reset timer
            12 -> {
                if (!timerState) {
                    pathTimer.resetTimer()
                    timerState = true
                }
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    // Reset centering flags before starting
                    centeringStarted = false
                    centeringComplete = false
                    isCentering = false
                    setPathState(13)
                }
            }

            // Center on depo for final score
            13 -> {
                intake = 0

                if (!centeringStarted) {
                    centeringStarted = true
                    pathTimer.resetTimer()
                }

                // Only attempt centering if follower is not busy
                if (notBusy && !isCentering) {
                    val centered = centerDepo()
                    if (centered) {
                        centeringComplete = true
                        setPathState(14)
                    }
                }

                // Timeout fallback
                if (pathTimer.elapsedTimeSeconds > centeringTimeoutSeconds) {
                    panels?.debug("Centering", "Timeout - proceeding anyway")
                    centeringComplete = true
                    setPathState(14)
                }
            }

            // Execute final dispense
            14 -> {
                if (!isDispensing) {
                    isDispensing = true
                    try {
                        executeDispensing()
                    } catch (e: Exception) {
                        panels?.debug("Dispense Error", e.message ?: "Unknown")
                    } finally {
                        isDispensing = false
                        setPathState(15) // Go to done state, not back to 5
                    }
                }
            }

            // Done!
            15 -> {
                // Autonomous complete
                follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
            }
        }
    }

    fun handleIntake() {
        val isFull = ord.none { it == "N" }
        if (isFull) {
            intakeServo1.power = ServoPositions.INTAKE_REVERSE
        } else if (intake == 1) {
            intakeServo1.power = ServoPositions.INTAKE_ON
        } else {
            intakeServo1.power = ServoPositions.INTAKE_OFF
        }
    }

    suspend fun handleDetections() {
        if (isDispensing) return

        val r = colorSensor.red()
        val g = colorSensor.green()
        val b = colorSensor.blue()

        if (g <= 80 && b <= 110) {
            isSeen = false
        }

        if (g >= 80 && b >= 110) {
            if (!isSeen) {
                val slot = nextSlot()
                if (slot != -1) {
                    ord[slot] = "P"
                    advanceBowl(slot)
                    delay(Timing.DETECTION_COOLDOWN)
                }
                isSeen = true
            }
        }

        if (g >= 110) {
            if (!isSeen) {
                val slot = nextSlot()
                if (slot != -1) {
                    ord[slot] = "G"
                    advanceBowl(slot)
                    delay(Timing.DETECTION_COOLDOWN)
                }
                isSeen = true
            }
        }

        panels?.debug("ord[0]", ord[0])
        panels?.debug("ord[1]", ord[1])
        panels?.debug("ord[2]", ord[2])
    }

    fun nextSlot(): Int {
        return when {
            ord[0] == "N" -> 0
            ord[1] == "N" -> 1
            ord[2] == "N" -> 2
            else -> -1
        }
    }

    fun advanceBowl(slot: Int) {
        bowlServo.position = when (slot) {
            0 -> ServoPositions.LOAD_P2
            1 -> ServoPositions.LOAD_P3
            2 -> ServoPositions.FIRE_P2
            else -> bowlServo.position
        }
    }

    suspend fun executeDispensing() {
        val isFull = ord.none { it == "N" }

        if (isFull) {
            // Fixed dispense sequence - no sorting needed
            val dispenseSequence = listOf(
                ServoPositions.FIRE_P2,
                ServoPositions.FIRE_P1,
                ServoPositions.FIRE_P3
            )
            reCenterDepo()
            executeDispenseSequence(dispenseSequence)
        }

        // Reset
        delay(100)
        bowlServo.position = ServoPositions.LOAD_P1
        delay(100)

        ord = arrayOf("N", "N", "N")
    }

    suspend fun executeDispenseSequence(positions: List<Double>) {
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

    suspend fun reCenterDepo() {
        follower.setMaxPower(0.25)

        try {
            val result: LLResult? = limelight.latestResult

            if (result == null || !result.isValid) {
                follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
                return
            }

            val fiducialResults = result.fiducialResults
            val target = fiducialResults?.firstOrNull { it.fiducialId == AprilTagIds.RED_DEPO }

            if (target == null) {
                follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
                return
            }

            val xErrPx: Double = target.targetXPixels - (DepoCenter.CAM_WIDTH_PX / 2.0)

            if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
                follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
                return
            }

            val rotationPower = clip(xErrPx * DepoCenter.KP_ROTATE, -0.3, 0.3)
            follower.setTeleOpDrive(0.0, 0.0, -rotationPower, false)
        } catch (e: Exception) {
            panels?.debug("ReCenter Error", e.message ?: "Unknown")
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
        }
    }

    /**
     * Centers on the DEPO AprilTag.
     * Returns true if centered, false if still adjusting or no target found.
     * Now uses a single turnTo call and waits for completion.
     */
    fun centerDepo(): Boolean {
        // Prevent re-entry while a turn is in progress
        if (isCentering) {
            // Check if the previous turn completed
            if (!follower.isBusy) {
                isCentering = false
                // Will re-evaluate on next call
            }
            return false
        }

        try {
            val result: LLResult? = limelight.latestResult

            if (result == null || !result.isValid) {
                panels?.debug("CenterDepo", "No valid result")
                return false
            }

            val fiducialResults = result.fiducialResults

            if (fiducialResults == null || fiducialResults.isEmpty()) {
                panels?.debug("CenterDepo", "No fiducials found")
                return false
            }

            val target = fiducialResults.firstOrNull { it.fiducialId == AprilTagIds.RED_DEPO }

            if (target == null) {
                panels?.debug("CenterDepo", "RED_DEPO tag not found")
                return false
            }

            val xErrPx: Double = target.targetXPixels - (DepoCenter.CAM_WIDTH_PX / 2.0)
            panels?.debug("CenterDepo X Error", xErrPx)

            if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
                panels?.debug("CenterDepo", "Centered!")
                return true
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

            panels?.debug("CenterDepo Turn", Math.toDegrees(turnStep))

            // Set flag before calling turnTo to prevent re-entry
            isCentering = true
            follower.turnTo(targetHeading)

            return false

        } catch (e: Exception) {
            panels?.debug("CenterDepo Error", e.message ?: "Unknown")
            isCentering = false
            return false
        }
    }

    fun outTakePower() {
        try {
            val result: LLResult? = limelight.latestResult

            if (result == null || !result.isValid) {
                return
            }

            val fiducialResults = result.fiducialResults

            if (fiducialResults == null || fiducialResults.isEmpty()) {
                return
            }

            val target = fiducialResults.firstOrNull { it.fiducialId == AprilTagIds.RED_DEPO }

            if (target == null) {
                return
            }

            val targetY = target.targetYPixels
            val powerResult = 0.0002416 * targetY + 0.105
            DepoCenter.OUTTAKE_SPEED = powerResult
            outTake1.power = DepoCenter.OUTTAKE_SPEED
            outTake2.power = DepoCenter.OUTTAKE_SPEED
        } catch (e: Exception) {
            panels?.debug("OutTakePower Error", e.message ?: "Unknown")
        }
    }

    fun initializeHardware() {
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
        bowlServo.position = ServoPositions.FIRE_P2

        setupMotorDirections()
        setupPIDFCoefficients()
        resetEncoders()
        setupVision()
    }

    fun setupVision() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
    }

    fun setupMotorDirections() {
        listOf(outTake2, liftLeft)
            .forEach { it.direction = DcMotorSimple.Direction.REVERSE }
    }

    fun setupPIDFCoefficients() {
        listOf(outTake1, outTake2)
            .forEach { it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF) }
    }

    fun resetEncoders() {
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

    fun initializePedroPathing() {
        panels = PanelsTelemetry.telemetry

        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()

        ord[0] = "P"
        ord[1] = "P"
        ord[2] = "P"

        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()
    }

    fun ensureVelocityMode() {
        if (!velocityModeInitialized) {
            outTake1.mode = DcMotor.RunMode.RUN_USING_ENCODER
            outTake2.mode = DcMotor.RunMode.RUN_USING_ENCODER
            velocityModeInitialized = true
        }
    }

    fun powerToTicksPerSecond(motor: DcMotorEx, power: Double): Double {
        val clipped = max(-1.0, min(1.0, power))
        val maxRpm = motor.motorType.maxRPM
        val tpr = motor.motorType.ticksPerRev
        val maxTicksPerSec = (maxRpm * tpr) / 60.0
        return clipped * velocityPowerScale * maxTicksPerSec
    }

    fun setMotorVelocityFromPseudoPower(motor: DcMotorEx, power: Double) {
        ensureVelocityMode()
        motor.velocity = powerToTicksPerSecond(motor, power)
    }

    fun clip(v: Double, min: Double, max: Double): Double {
        return max(min, min(max, v))
    }

    @JvmName("SetPathStateFunction")
    fun setPathState(pState: Int) {
        pathState = pState
        pathTimer.resetTimer()
        timerState = false
    }
}