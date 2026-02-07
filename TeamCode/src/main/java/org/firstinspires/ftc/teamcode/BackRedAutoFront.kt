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

@Autonomous(name = "Back Red Auto (12 front)", group = "Main Red")
class BackRedAutoFront : OpMode() {
    var panels: TelemetryManager? = null
    val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    var runDetections: Job? = null
    var outTakeCalc: Job? = null
    var correction: Job? = null
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
    @Volatile var finalShot = 0
    @Volatile var cycleCount = 4

    private val startPose    = Pose(72.0,  0.0,   Math.toRadians(90.0))
    private val scorePose    = Pose(74.0,  2.0,   Math.toRadians(60.0))
    private val spikeScore   = Pose(79.0,  12.0,  Math.toRadians(64.0))
    private val spikeScore2  = Pose(79.0,  13.0,  Math.toRadians(65.0))
    private val spikeScore3  = Pose(79.0,  14.0,  Math.toRadians(69.0))
    private val spike1pre    = Pose(85.9,  24.0,  Math.toRadians(0.0))
    private val spike1       = Pose(106.4, 24.0,  Math.toRadians(0.0))
    private val spike2pre    = Pose(88.0,  48.5,  Math.toRadians(0.0))
    private val spike2       = Pose(106.4, 48.5,  Math.toRadians(0.0))
    private val spike3pre    = Pose(88.0,  72.5,  Math.toRadians(0.0))
    private val spike3       = Pose(106.4, 72.5,  Math.toRadians(0.0))
    private val spike3fire   = Pose(79.0,  88.0, Math.toRadians(33.0))
    private val strafeOut    = Pose(100.0, 4.0,   Math.toRadians(90.0))

    private lateinit var preLoadScore: PathChain
    private lateinit var spike1Line:   PathChain
    private lateinit var spike1Grab:   PathChain
    private lateinit var spike1Score:  PathChain
    private lateinit var spike2Line:   PathChain
    private lateinit var spike2Grab:   PathChain
    private lateinit var spike2Score:  PathChain
    private lateinit var spike3Line:   PathChain
    private lateinit var spike3Grab:   PathChain
    private lateinit var spike3Score:  PathChain
    private lateinit var leavePoint:   PathChain
    val pidP = 150.0
    val pidI = 0.0
    val pidD = 0.0
    val pidF = 13.5
    var velocityModeInitialized = false
    var velocityPowerScale = 1.0

    object ServoPositions {
        const val LOAD_P1 = 0.021
        const val LOAD_P2 = 0.087
        const val LOAD_P3 = 0.158
        const val FIRE_P1 = 0.128
        const val FIRE_P2 = 0.195
        const val FIRE_P3 = 0.058
        const val CAM_OPEN = 0.44
        const val CAM_CLOSED = 0.255
        const val INTAKE_ON = 1.0
        const val INTAKE_OFF = 0.0
    }

    object AprilTagIds {
        const val RED_DEPO = 24
    }

    object DepoCenter {
        const val CAM_WIDTH_PX = 1280
        const val CENTER_DEADZONE = 13
        const val KP_ROTATE = 0.004
        var OUTTAKE_SPEED = 0.20
    }

    object Timing {
        const val BOWL_MOVE_DELAY = 250L
        const val CAM_OPEN_DELAY = 140L
        const val CAM_CLOSE_DELAY = 170L
        const val DETECTION_COOLDOWN = 400L
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
        follower.startTeleOpDrive()
        outTakeCalc = scope.launch {
            while (isActive) {
                outTakePower()
                delay(5)
            }
        }
        runDetections = scope.launch {
            while (isActive) {
                handleDetections()
                delay(3)
            }
        }
        resetRuntime()
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()
        handleIntake()
        Drawing.drawDebug(follower)
        if (finalShot == cycleCount) {
            outTake1.power = 0.0
            outTake2.power = 0.0
            outTakeCalc?.cancel()
        }
        panels?.debug("final shot", finalShot)
        panels?.debug("Path State", pathState)
        /*panels?.debug("ord[0]", ord[0])
        panels?.debug("ord[1]", ord[1])
        panels?.debug("ord[2]", ord[2])*/
        panels?.debug("X", follower.pose.x)
        panels?.debug("Y", follower.pose.y)
        panels?.debug("H", Math.toDegrees(follower.pose.heading))
        /*panels?.debug("follower speed", follower.velocity)
        panels?.debug("follower distance Remaining", follower.distanceRemaining)
        panels?.debug("follower distance Traveled On Path", follower.distanceTraveledOnPath)*/
        panels?.debug("Timer", pathTimer.elapsedTimeSeconds)
        panels?.debug("RunTime", runtime)
        panels?.update(telemetry)
    }

    override fun stop() {
        runDetections?.cancel()
        outTakeCalc?.cancel()
        slowDown?.cancel()
        correction?.cancel()
        limelight.stop()
    }

    fun autonomousPathUpdate() {
        val notBusy = !follower.isBusy
        when (pathState) {
            0 -> {
                if (notBusy && !timerState) {
                    pathTimer.resetTimer()
                    timerState = true
                    follower.followPath(preLoadScore, false)
                    setPathState(1)
                }
            }
            1 -> {
                follower.setMaxPower(0.9)
                if (notBusy) {
                    setPathState(2)
                }
            }
            2 -> {
                if (pathTimer.elapsedTimeSeconds > 0.01) {
                    setPathState(3)
                }
            }
            3 -> {
                val centered = centerDepo()
                if (/*centered ||*/ pathTimer.elapsedTimeSeconds > 0.0) {
                    setPathState(4)
                }
            }
            4 -> {
                if (!isDispensing) {
                    isDispensing = true
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true)
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
                follower.setMaxPower(0.9)
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.01) {
                        follower.followPath(spike1Grab, false)
                        setPathState(7)
                    }
                }
            }
            7 -> {
                follower.setMaxPower(0.2199)
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.01) {
                        follower.followPath(spike1Score, false)
                        setPathState(8)
                    }
                }
            }
            8 -> {
                /*if (correction?.isActive != true) {
                    correction = scope.launch {
                        while (isActive) {
                            if (follower.pose.y < 12.0) follower.breakFollowing()
                            delay(15)
                        }
                    }
                }*/
                follower.setMaxPower(0.9)
                if (notBusy) {
                    if (correction?.isActive == true) {
                        correction?.cancel()
                    }
                    setPathState(9)
                }
            }
            9 -> {
                correction?.cancel()
                if (pathTimer.elapsedTimeSeconds > 0.01) {
                    setPathState(10)
                }
            }
            10 -> {
                val centered = centerDepo()
                if (/*centered ||*/ pathTimer.elapsedTimeSeconds > 0.0) {
                    setPathState(11)
                }
            }
            11 -> {
                if (!isDispensing) {
                    isDispensing = true
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true)
                    scope.launch {
                        executeDispensing()
                        isDispensing = false
                        setPathState(12)
                    }
                }
            }
            12 -> {
                if (!isDispensing) {
                    follower.breakFollowing()
                    follower.followPath(spike2Line, false)
                    setPathState(13)
                }
            }
            13 -> {
                follower.setMaxPower(0.9)
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.01) {
                        follower.followPath(spike2Grab, false)
                        setPathState(14)
                    }
                }
            }
            14 -> {
                follower.setMaxPower(0.2199)
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.01) {
                        follower.followPath(spike2Score, false)
                        setPathState(15)
                    }
                }
            }
            15 -> {
                /*if (correction?.isActive != true) {
                    correction = scope.launch {
                        while (isActive) {
                            if (follower.pose.y < 16.0) follower.breakFollowing()
                            delay(15)
                        }
                    }
                }*/
                follower.setMaxPower(0.9)
                if (notBusy) {
                    if (correction?.isActive == true) {
                        correction?.cancel()
                    }
                    setPathState(16)
                }
            }
            16 -> {
                correction?.cancel()
                if (pathTimer.elapsedTimeSeconds > 0.01) {
                    setPathState(17)
                }
            }
            17 -> {
                val centered = centerDepo()
                if (/*centered ||*/ pathTimer.elapsedTimeSeconds > 0.0) {
                    setPathState(18)
                }
            }
            18 -> {
                if (!isDispensing) {
                    isDispensing = true
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true)
                    scope.launch {
                        executeDispensing()
                        isDispensing = false
                        setPathState(19)
                    }
                }
            }
            19 -> {
                if (!isDispensing) {
                    follower.breakFollowing()
                    follower.followPath(spike3Line, true)
                    setPathState(20)
                }
            }
            20 -> {
                follower.setMaxPower(0.9)
                /*if (correction?.isActive != true) {
                    correction = scope.launch {
                        while (isActive) {
                            if (follower.pose.y > 62.0) follower.breakFollowing()
                            delay(15)
                        }
                    }
                }*/
                if (notBusy) {
                    if (correction?.isActive == true) {
                        correction?.cancel()
                    }
                    correction?.cancel()
                    if (!timerState) {
                        correction?.cancel()
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.01) {
                        correction?.cancel()
                        follower.followPath(spike3Grab, false)
                        setPathState(21)
                    }
                }
            }
            21 -> {
                follower.setMaxPower(0.2199)
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.01) {
                        follower.followPath(spike3Score, false)
                        setPathState(22)
                    }
                }
            }
            22 -> {
                /*if (correction?.isActive != true) {
                    correction = scope.launch {
                        while (isActive) {
                            if (follower.pose.y < 16.0) follower.breakFollowing()
                            delay(15)
                        }
                    }
                }*/
                follower.setMaxPower(0.9)
                if (notBusy) {
                    if (correction?.isActive == true) {
                        correction?.cancel()
                    }
                    setPathState(23)
                }
            }
            23 -> {
                correction?.cancel()
                val centered = centerDepo()
                if (/*centered ||*/ pathTimer.elapsedTimeSeconds > 0.0) {
                    setPathState(24)
                }
            }
            24 -> {
                if (!isDispensing) {
                    isDispensing = true
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true)
                    scope.launch {
                        executeDispensing()
                        isDispensing = false
                        //setPathState(25)
                    }
                }
            }
            25 -> {
                if (!isDispensing) {
                    follower.breakFollowing()
                    follower.followPath(leavePoint, false)
                    setPathState(26)
                }
            }
            26 -> {
                follower.setMaxPower(0.9)
            }
        }
    }

    fun centerDepo(): Boolean {
        follower.setMaxPower(0.15)
        val result: LLResult? = limelight.latestResult
        val fiducialResults = result?.fiducialResults
        val target = fiducialResults?.firstOrNull { it.fiducialId == AprilTagIds.RED_DEPO }

        if (target == null) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
            return false
        }

        val xErrPx: Double = target.targetXPixels - (DepoCenter.CAM_WIDTH_PX / 2.0)

        if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
            return true
        }

        val rotationPower = clip(xErrPx * DepoCenter.KP_ROTATE, -0.3, 0.3)
        follower.setTeleOpDrive(0.0, 0.0, -rotationPower, false)
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
        finalShot += 1
    }

    suspend fun executeDispenseSequence(positions: List<Double>) {
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
        val target = fiducialResults.firstOrNull { it.fiducialId == AprilTagIds.RED_DEPO }
        if (target == null) {
            return
        }
        val targetY = target.targetYPixels
        //val powerResult = if (runtime >= 25.0) 0.0002416 * targetY + 0.120 else 0.0002416 * targetY + 0.115
        var powerResult = if (runtime >= 25.0) 0.0002416 * targetY + 0.120 else 0.0002416 * targetY + 0.105
        DepoCenter.OUTTAKE_SPEED = powerResult
        DepoCenter.OUTTAKE_SPEED = powerResult
        outTake1.power = DepoCenter.OUTTAKE_SPEED
        outTake2.power = DepoCenter.OUTTAKE_SPEED
    }

    fun handleIntake() {
        /*val isFull = ord.none { it == "N" }  // True if all 3 slots filled

        if (isFull || isDispensing) {
            intakeServo1.power = -ServoPositions.INTAKE_ON  // Reverse/outtake
        } else {
            intakeServo1.power = ServoPositions.INTAKE_ON   // Intake
        }*/
        intakeServo1.power = ServoPositions.INTAKE_ON
    }

    suspend fun handleDetections() {
        if (isDispensing) return
        val r = colorSensor.red();val g = colorSensor.green();val b = colorSensor.blue()

        if (g <= 80 && b <= 110) {
            isSeen = false
            return
        }

        val now = System.currentTimeMillis()
        if (now < Timing.nextDetectAllowedMs) return

        if (!isSeen) {
            val slot = nextSlot()
            if (slot != -1) {
                ord[slot] = if (g >= 110) "G" else "P"

                delay(200)
                advanceBowl(slot)

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
            .addPath(BezierCurve(spike1, spikeScore))
            .setLinearHeadingInterpolation(spike1.heading, spikeScore.heading)
            .build()

        spike2Line = follower.pathBuilder()
            .addPath(BezierCurve(spikeScore, spike2pre))
            .setLinearHeadingInterpolation(spikeScore.heading, spike2pre.heading)
            .build()

        spike2Grab = follower.pathBuilder()
            .addPath(BezierCurve(spike2pre, spike2))
            .setLinearHeadingInterpolation(spike2pre.heading, spike2.heading)
            .build()
        spike2Score = follower.pathBuilder()
            .addPath(BezierCurve(spike2, spikeScore2))
            .setLinearHeadingInterpolation(spike2.heading, spikeScore2.heading)
            .build()
        spike3Line = follower.pathBuilder()
            .addPath(BezierCurve(spikeScore2, spike3pre))
            .setLinearHeadingInterpolation(spikeScore2.heading, spike3pre.heading)
            .build()
        spike3Grab = follower.pathBuilder()
            .addPath(BezierCurve(spike3pre, spike3))
            .setLinearHeadingInterpolation(spike3pre.heading, spike3.heading)
            .build()
        spike3Score = follower.pathBuilder()
            .addPath(BezierCurve(spike3, spike3fire))
            .setLinearHeadingInterpolation(spike3.heading, spike3fire.heading)
            .build()
        leavePoint = follower.pathBuilder()
            .addPath(BezierCurve(spikeScore3, spike2pre))
            .setLinearHeadingInterpolation(spikeScore3.heading, spike2pre.heading)
            .build()
    }

    @JvmName("SetPathStateFunction")
    private fun setPathState(pState: Int) {
        pathState = pState
        pathTimer.resetTimer()
        timerState = false
    }
    fun clip(v: Double, min: Double, max: Double): Double {
        return max(min, min(max, v))
    }
}