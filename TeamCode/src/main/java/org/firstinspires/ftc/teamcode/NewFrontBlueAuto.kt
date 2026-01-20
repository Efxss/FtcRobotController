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
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
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
import org.firstinspires.ftc.teamcode.util.Drawing
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

@Autonomous(name = "Front Blue Auto", group = "Main Blue")
class NewFrontBlueAuto : OpMode() {
    var panels: TelemetryManager? = null
    val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    var runDetections: Job? = null
    var outTakeCalc: Job? = null
    var slowDown: Job? = null

    @Volatile lateinit var follower: Follower
    @Volatile lateinit var pathTimer: Timer
    @Volatile lateinit var actionTimer: Timer
    @Volatile lateinit var opmodeTimer: Timer

    lateinit var outTake1: DcMotorEx
    lateinit var outTake2: DcMotorEx
    lateinit var intakeServo1: CRServo
    lateinit var bowlServo: Servo
    lateinit var camServo: Servo
    lateinit var limelight: Limelight3A
    lateinit var colorSensor: ColorSensor

    @Volatile var ord = arrayOf("N", "N", "N")
    @set:JvmName("PathStateVar")
    var pathState: Int = 0
    var timerState = false
    var isSeen = false
    @Volatile var isDispensing = false

    // Poses
    private val startPose    = Pose(  3.0, 123.0, Math.toRadians(140.0))
    private val scorePose    = Pose( 34.5,  91.5, Math.toRadians(135.0))
    private val spike1pre    = Pose( 22.0,  86.0, Math.toRadians(180.0))
    private val spike1       = Pose(  5.0,  86.0, Math.toRadians(180.0))
    private val spike2pre    = Pose( 20.0,  65.0, Math.toRadians(180.0))
    private val spike2       = Pose(  5.0,  65.0, Math.toRadians(180.0))

    // Paths
    private lateinit var preLoadScore: PathChain
    private lateinit var spike1Line:   PathChain
    private lateinit var spike1Grab:   PathChain
    private lateinit var spike1Score:  PathChain
    private lateinit var spike2Line:   PathChain
    private lateinit var spike2Grab:   PathChain

    val pidP = 150.0
    val pidI = 0.0
    val pidD = 0.0
    val pidF = 13.5
    var velocityModeInitialized = false
    var velocityPowerScale = 1.0

    object ServoPositions {
        // Loading positions
        const val LOAD_P1 = 0.021
        const val LOAD_P2 = 0.087
        const val LOAD_P3 = 0.158

        // Firing/dispensing positions
        const val FIRE_P1 = 0.128
        const val FIRE_P2 = 0.195
        const val FIRE_P3 = 0.058

        // Camera servo positions
        const val CAM_OPEN = 0.44
        const val CAM_CLOSED = 0.255

        const val INTAKE_ON = 1.0
        const val INTAKE_OFF = 0.0
    }

    object AprilTagIds {
        const val BLUE_DEPO = 20
    }

    object DepoCenter {
        const val CAM_WIDTH_PX = 1280
        const val CENTER_DEADZONE = 15
        const val KP_ROTATE = 0.003
        var OUTTAKE_SPEED = 0.20
    }

    object Timing {
        const val DISPENSE_INITIAL_DELAY = 100L
        const val BOWL_MOVE_DELAY = 250L
        const val CAM_OPEN_DELAY = 140L
        const val CAM_CLOSE_DELAY = 170L
        const val DETECTION_COOLDOWN = 1200L
        var nextDetectAllowedMs = 0L
    }
    override fun init() {
        initializeHardware()
        initializePedroPathing()
    }

    override fun init_loop() {
        follower.update()
        Drawing.drawOnlyCurrent(follower)
    }

    override fun start() {
        opmodeTimer.resetTimer()
        setPathState(0)

        outTakeCalc = scope.launch {
            while (isActive) {
                outTakePower()
                delay(5)
            }
        }

        runDetections = scope.launch {
            while (isActive) {
                handleDetections()
                delay(25)
            }
        }
        resetRuntime()
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()
        handleIntake()
        Drawing.drawDebug(follower)
        panels?.debug("Path State", pathState)
        panels?.debug("ord[0]", ord[0])
        panels?.debug("ord[1]", ord[1])
        panels?.debug("ord[2]", ord[2])
        panels?.debug("Timer", pathTimer.elapsedTimeSeconds)
        panels?.debug("RunTime", runtime)
        panels?.update(telemetry)
        if (runtime >= 29.0) {
            terminateOpModeNow()
        }
    }

    override fun stop() {
        runDetections?.cancel()
        outTakeCalc?.cancel()
        slowDown?.cancel()
        limelight.stop()
    }

    fun autonomousPathUpdate() {
        val notBusy = !follower.isBusy
        when (pathState) {
            0 -> {
                if (notBusy && !timerState) {
                    pathTimer.resetTimer()
                    timerState = true
                    follower.followPath(preLoadScore, true)
                    setPathState(1)
                }
            }
            1 -> {
                follower.setMaxPower(0.7)
                if (notBusy) {
                    setPathState(2)
                }
            }
            2 -> {
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    setPathState(3)
                }
            }
            3 -> {
                val centered = centerDepo()
                if (centered || pathTimer.elapsedTimeSeconds > 0.1) {
                    setPathState(4)
                }
            }
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
            5 -> {
                if (!isDispensing) {
                    follower.breakFollowing()
                    follower.followPath(spike1Line, false)
                    setPathState(6)
                }
            }
            6 -> {
                follower.setMaxPower(0.40)
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.3) {
                        follower.followPath(spike1Grab, true)
                        setPathState(7)
                    }
                }
            }
            7 -> {
                follower.setMaxPower(0.16)
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.3) {
                        follower.followPath(spike1Score, true)
                        setPathState(8)
                    }
                }
            }
            8 -> {
                follower.setMaxPower(0.8)
                if (notBusy) {
                    setPathState(9)
                }
            }
            9 -> {
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    setPathState(10)
                }
            }
            10 -> {
                val centered = centerDepo()
                if (centered || pathTimer.elapsedTimeSeconds > 0.1) {
                    setPathState(11)
                }
            }
            11 -> {
                if (!isDispensing) {
                    isDispensing = true
                    scope.launch {
                        executeDispensing()
                        isDispensing = false
                        outTakeCalc?.cancel();outTake1.power=0.0;outTake2.power=0.0
                        setPathState(12)
                    }
                }
            }
            12 -> {
                follower.setMaxPower(0.35)
                if (!isDispensing) {
                    follower.breakFollowing()
                    follower.followPath(spike2Line, false)
                    setPathState(13)
                }
            }
            13 -> {
                follower.setMaxPower(0.47)
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.3) {
                        slowDown = scope.launch { while (isActive) { if (ord[0] != "N") follower.setMaxPower(0.15) } }
                        follower.followPath(spike2Grab, true)
                        setPathState(14)
                    }
                }
            }
            14 -> {
                follower.setMaxPower(0.18)
            }
        }
    }

    private fun centerDepo(): Boolean {
        follower.setMaxPower(0.8)
        val result: LLResult? = limelight.latestResult
        val fiducialResults = result?.fiducialResults
        val target = fiducialResults?.firstOrNull { it.fiducialId == AprilTagIds.BLUE_DEPO }

        if (target == null) {
            return false
        }

        val xErrPx: Double = target.targetXPixels - (DepoCenter.CAM_WIDTH_PX / 2.0)

        // Already centered
        if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
            return true
        }

        // Convert pixel error to angle
        val hFovDeg = 70.0  // Limelight 3A horizontal FOV
        val hFovRad = Math.toRadians(hFovDeg)
        val pixelsToRad = hFovRad / DepoCenter.CAM_WIDTH_PX
        val angleError = xErrPx * pixelsToRad

        // Calculate target heading
        val currentHeading = follower.pose.heading
        val targetHeading = currentHeading - angleError

        // Use PedroPathing's turnTo
        follower.turnTo(targetHeading)

        return false
    }

    suspend fun executeDispensing() {
        // Fixed dispense order: slot 1, slot 0, slot 2 (FIRE_P2, FIRE_P1, FIRE_P3)
        val dispenseSequence = mutableListOf<Double>()
        val slotToFirePosition = mapOf(
            0 to ServoPositions.FIRE_P1,
            1 to ServoPositions.FIRE_P2,
            2 to ServoPositions.FIRE_P3
        )
        val dispenseOrder = listOf(1, 0, 2)

        for (slotIndex in dispenseOrder) {
            if (ord[slotIndex] != "N") {
                slotToFirePosition[slotIndex]?.let { dispenseSequence.add(it) }
            }
        }

        if (dispenseSequence.isNotEmpty()) {
            executeDispenseSequence(dispenseSequence)
        }

        bowlServo.position = ServoPositions.LOAD_P1

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

    fun outTakePower() {
        val result: LLResult? = limelight.latestResult
        if (result == null || !result.isValid) {
            return
        }
        val fiducialResults = result.fiducialResults
        val target = fiducialResults.firstOrNull { it.fiducialId == AprilTagIds.BLUE_DEPO }
        if (target == null) {
            return
        }
        val targetY = target.targetYPixels
        val powerResult = 0.0002416 * targetY + 0.110
        DepoCenter.OUTTAKE_SPEED = powerResult
        outTake1.power = DepoCenter.OUTTAKE_SPEED
        outTake2.power = DepoCenter.OUTTAKE_SPEED
    }

    fun handleIntake() {
        val isFull = ord.none { it == "N" }  // True if all 3 slots filled

        if (isFull || isDispensing) {
            intakeServo1.power = -ServoPositions.INTAKE_ON  // Reverse/outtake
        } else {
            intakeServo1.power = ServoPositions.INTAKE_ON   // Intake
        }
    }

    suspend fun handleDetections() {
        if (isDispensing) return
        val r = colorSensor.red();val g = colorSensor.green();val b = colorSensor.blue()

        // "Clear" => re-arm immediately (even during cooldown)
        if (g <= 80 && b <= 110) {
            isSeen = false
            return
        }

        // If we're still in cooldown, don't trigger a new advance yet
        val now = System.currentTimeMillis()
        if (now < Timing.nextDetectAllowedMs) return

        // Ball present and we're armed
        if (!isSeen) {
            val slot = nextSlot()
            if (slot != -1) {
                // (Optional) choose G vs P here; your current logic makes g>=110 also count as P first.
                ord[slot] = if (g >= 110) "G" else "P"

                // keep the short settle delay if you need it
                delay(200)
                advanceBowl(slot)

                // start cooldown WITHOUT blocking the loop
                Timing.nextDetectAllowedMs = now + Timing.DETECTION_COOLDOWN
            }
            isSeen = true
        }
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

    fun initializeHardware() {
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
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

    fun followerSpeed(speed: Double) {
        follower.setMaxPower(speed)
    }

    fun setupVision() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
        limelight.start()
    }

    fun setupMotorDirections() {
        outTake2.direction = DcMotorSimple.Direction.REVERSE
    }

    fun setupPIDFCoefficients() {
        listOf(outTake1, outTake2).forEach {
            it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF)
        }
    }

    fun resetEncoders() {
        listOf(outTake1, outTake2).forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    fun initializePedroPathing() {
        panels = PanelsTelemetry.telemetry

        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()

        val otos = hardwareMap.get(SparkFunOTOS::class.java, "otos")
        otos.calibrateImu()
        otos.resetTracking()

        ord = arrayOf("P", "G", "P")

        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()

        buildPaths()
        Drawing.init()
    }

    fun buildPaths() {
        preLoadScore = follower.pathBuilder()
            .addPath(BezierCurve(startPose, scorePose))
            .setLinearHeadingInterpolation(startPose.heading, scorePose.heading)
            .build()

        spike1Line = follower.pathBuilder()
            .addPath(BezierCurve(scorePose, spike1pre))
            .setLinearHeadingInterpolation(scorePose.heading, spike1pre.heading)
            .build()

        spike1Grab = follower.pathBuilder()
            .addPath(BezierCurve(spike1pre, spike1))
            .setLinearHeadingInterpolation(spike1pre.heading, spike1.heading)
            .build()

        spike1Score = follower.pathBuilder()
            .addPath(BezierCurve(spike1, scorePose))
            .setLinearHeadingInterpolation(spike1.heading, scorePose.heading)
            .build()

        spike2Line = follower.pathBuilder()
            .addPath(BezierCurve(scorePose, spike2pre))
            .setLinearHeadingInterpolation(scorePose.heading, spike2pre.heading)
            .build()

        spike2Grab = follower.pathBuilder()
            .addPath(BezierCurve(spike2pre, spike2))
            .setLinearHeadingInterpolation(spike2pre.heading, spike2.heading)
            .build()
    }

    fun clip(v: Double, min: Double, max: Double): Double {
        return max(min, min(max, v))
    }

    @JvmName("SetPathStateFunction")
    private fun setPathState(pState: Int) {
        pathState = pState
        pathTimer.resetTimer()
        timerState = false
    }
}
