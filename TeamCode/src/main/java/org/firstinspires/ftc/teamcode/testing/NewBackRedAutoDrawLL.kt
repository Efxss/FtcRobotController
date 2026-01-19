package org.firstinspires.ftc.teamcode.testing

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.util.Drawing
import org.firstinspires.ftc.teamcode.util.LimelightUtil

@Autonomous(name = "Back Red Auto (NEW DRAW LL)", group = "Main Red")
class NewBackRedAutoDrawLL : OpMode() {

    // Telemetry & Coroutines
    var panels: TelemetryManager? = null
    val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    var runDetections: Job? = null
    var outTakeCalc: Job? = null
    var slowDown: Job? = null

    // Pedro Pathing
    @Volatile lateinit var follower: Follower
    @Volatile lateinit var pathTimer: Timer
    @Volatile lateinit var actionTimer: Timer
    @Volatile lateinit var opmodeTimer: Timer

    // Hardware
    lateinit var outTake1: DcMotorEx
    lateinit var outTake2: DcMotorEx
    lateinit var intakeServo1: CRServo
    lateinit var colorSensor: ColorSensor

    // State tracking
    @Volatile var ord = arrayOf("N", "N", "N")
    @set:JvmName("PathStateVar")
    var pathState: Int = 0
    var timerState = false
    var isSeen = false
    @Volatile var isDispensing = false

    // Poses
    private val startPose    = Pose(72.0, 0.0, Math.toRadians(90.0))
    private val preloadPose  = Pose(74.0, 4.0, Math.toRadians(64.0))
    private val pickupPoint5 = Pose(80.0, 17.5, Math.toRadians(9.0))
    private val pickup1      = Pose(86.5, 24.0, Math.toRadians(0.0))
    private val pickup1Ball1 = Pose(93.3, 24.0, Math.toRadians(0.0))
    private val pickup1Ball2 = Pose(98.3, 24.0, Math.toRadians(0.0))
    private val pickup1Ball3 = Pose(104.8, 24.0, Math.toRadians(0.0))
    private val scoreBack    = Pose(78.0, 10.0, Math.toRadians(68.0))
    private val spike2       = Pose(89.0, 46.0, Math.toRadians(0.0))
    private val spike2Balls  = Pose(104.8, 48.0, Math.toRadians(0.0))

    // Paths
    private lateinit var preloadPose1: PathChain
    private lateinit var pickupPosePoint5: PathChain
    private lateinit var pickupPose1: PathChain
    private lateinit var pickupPose1Ball1: PathChain
    private lateinit var pickupPose1Ball2: PathChain
    private lateinit var pickupPose1Ball3: PathChain
    private lateinit var returnPose: PathChain
    private lateinit var stripe2: PathChain
    private lateinit var stripe2Grab: PathChain

    // Motor config
    val pidP = 150.0
    val pidI = 0.0
    val pidD = 0.0
    val pidF = 13.5

    // Detection timing
    object Timing {
        const val DETECTION_COOLDOWN = 1200L
        var nextDetectAllowedMs = 0L
    }

    override fun init() {
        initializeHardware()
        initializePedroPathing()

        // Initialize utilities
        Drawing.init()
        LimelightUtil.init(hardwareMap)
    }

    override fun init_loop() {
        follower.update()
        Drawing.drawOnlyCurrent(follower)

        panels?.debug("Status", "Ready to start")
        panels?.debug("Start Pose", "${startPose.x}, ${startPose.y}")
        panels?.update(telemetry)
    }

    override fun start() {
        opmodeTimer.resetTimer()
        setPathState(0)

        // Outtake power calculation coroutine
        outTakeCalc = scope.launch {
            while (isActive) {
                updateOuttakePower()
                delay(5)
            }
        }

        // Ball detection coroutine
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

        // Draw robot, path, and history
        Drawing.drawDebug(follower)

        // Telemetry
        panels?.debug("Path State", pathState)
        panels?.debug("X", follower.pose.x)
        panels?.debug("Y", follower.pose.y)
        panels?.debug("Heading", Math.toDegrees(follower.pose.heading))
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
        LimelightUtil.stop()
    }

    private fun buildPaths() {
        preloadPose1 = follower.pathBuilder()
            .addPath(BezierCurve(startPose, preloadPose))
            .setLinearHeadingInterpolation(startPose.heading, preloadPose.heading)
            .build()

        pickupPosePoint5 = follower.pathBuilder()
            .addPath(BezierCurve(preloadPose, pickupPoint5))
            .setLinearHeadingInterpolation(preloadPose.heading, pickupPoint5.heading)
            .build()

        pickupPose1 = follower.pathBuilder()
            .addPath(BezierCurve(pickupPoint5, pickup1))
            .setLinearHeadingInterpolation(pickupPoint5.heading, pickup1.heading)
            .build()

        pickupPose1Ball1 = follower.pathBuilder()
            .addPath(BezierCurve(pickup1, pickup1Ball1))
            .setLinearHeadingInterpolation(pickup1.heading, pickup1Ball1.heading)
            .build()

        pickupPose1Ball2 = follower.pathBuilder()
            .addPath(BezierCurve(pickup1Ball1, pickup1Ball2))
            .setLinearHeadingInterpolation(pickup1Ball1.heading, pickup1Ball2.heading)
            .build()

        pickupPose1Ball3 = follower.pathBuilder()
            .addPath(BezierCurve(pickup1Ball2, pickup1Ball3))
            .setLinearHeadingInterpolation(pickup1Ball2.heading, pickup1Ball3.heading)
            .build()

        returnPose = follower.pathBuilder()
            .addPath(BezierCurve(pickup1Ball3, scoreBack))
            .setLinearHeadingInterpolation(pickup1Ball3.heading, scoreBack.heading)
            .build()

        stripe2 = follower.pathBuilder()
            .addPath(BezierCurve(scoreBack, spike2))
            .setLinearHeadingInterpolation(scoreBack.heading, spike2.heading)
            .build()

        stripe2Grab = follower.pathBuilder()
            .addPath(BezierCurve(spike2, spike2Balls))
            .setLinearHeadingInterpolation(spike2.heading, spike2Balls.heading)
            .build()
    }

    private fun autonomousPathUpdate() {
        val notBusy = !follower.isBusy

        when (pathState) {
            // Start -> Preload position
            0 -> {
                if (notBusy && !timerState) {
                    pathTimer.resetTimer()
                    timerState = true
                    follower.followPath(preloadPose1, true)
                    setPathState(1)
                }
            }

            // Wait for arrival at preload
            1 -> {
                if (notBusy) setPathState(2)
            }

            // Brief pause before centering
            2 -> {
                if (pathTimer.elapsedTimeSeconds > 0.25) setPathState(3)
            }

            // Center on depot and fire preload
            3 -> {
                if (!isDispensing) {
                    isDispensing = true
                    scope.launch {
                        LimelightUtil.centerAndFire(
                            LimelightUtil.Tags.RED_DEPO,
                            follower,
                            ord,
                            centerTimeoutMs = 500L
                        )
                        ord = arrayOf("N", "N", "N")
                        isDispensing = false
                        setPathState(4)
                    }
                }
            }

            // Go to pickup intermediate point
            4 -> {
                if (!isDispensing) {
                    follower.breakFollowing()
                    follower.setMaxPower(0.6)
                    follower.followPath(pickupPosePoint5, false)
                    setPathState(5)
                }
            }

            // Wait and go to pickup1
            5 -> {
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.3) {
                        follower.setMaxPower(0.4)
                        follower.followPath(pickupPose1, true)
                        setPathState(6)
                    }
                }
            }

            // Slow pickup - Ball 1
            6 -> {
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.2) {
                        follower.setMaxPower(0.18)
                        follower.followPath(pickupPose1Ball1, true)
                        setPathState(7)
                    }
                }
            }

            // Slow pickup - Ball 2
            7 -> {
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.2) {
                        follower.setMaxPower(0.2)
                        follower.followPath(pickupPose1Ball2, true)
                        setPathState(8)
                    }
                }
            }

            // Slow pickup - Ball 3
            8 -> {
                if (notBusy) {
                    if (!timerState) {
                        pathTimer.resetTimer()
                        timerState = true
                    }
                    if (pathTimer.elapsedTimeSeconds > 0.2) {
                        follower.setMaxPower(0.22)
                        follower.followPath(pickupPose1Ball3, true)
                        setPathState(9)
                    }
                }
            }

            // Return to scoring position
            9 -> {
                if (notBusy) {
                    follower.setMaxPower(0.6)
                    follower.followPath(returnPose, true)
                    setPathState(10)
                }
            }

            // Wait for arrival
            10 -> {
                if (notBusy) setPathState(11)
            }

            // Brief pause before second score
            11 -> {
                if (pathTimer.elapsedTimeSeconds > 0.25) setPathState(12)
            }

            // Center and fire second load
            12 -> {
                if (!isDispensing) {
                    isDispensing = true
                    scope.launch {
                        LimelightUtil.centerAndFire(
                            LimelightUtil.Tags.RED_DEPO,
                            follower,
                            ord,
                            centerTimeoutMs = 300L
                        )
                        ord = arrayOf("N", "N", "N")

                        // Stop outtake motors after firing
                        outTakeCalc?.cancel()
                        outTake1.power = 0.0
                        outTake2.power = 0.0

                        isDispensing = false
                        setPathState(13)
                    }
                }
            }

            // Go to spike 2
            13 -> {
                if (!isDispensing && (notBusy || pathTimer.elapsedTimeSeconds > 1.0)) {
                    follower.setMaxPower(0.6)
                    follower.followPath(stripe2, true)
                    setPathState(14)
                }
            }

            // Slow grab at spike 2
            14 -> {
                if (notBusy) {
                    follower.setMaxPower(0.18)
                    slowDown = scope.launch {
                        while (isActive) {
                            if (ord[0] != "N") follower.setMaxPower(0.16)
                        }
                    }
                    follower.followPath(stripe2Grab, true)
                    setPathState(15)
                }
            }

            // Done
            15 -> {
                if (notBusy) {
                    slowDown?.cancel()
                }
            }
        }
    }

    private fun updateOuttakePower() {
        val power = LimelightUtil.calculateOuttakePower(LimelightUtil.Tags.RED_DEPO)
        if (power != null) {
            outTake1.power = power
            outTake2.power = power
        }
    }

    private fun handleIntake() {
        val isFull = ord.none { it == "N" }

        intakeServo1.power = if (isFull || isDispensing) {
            -1.0  // Reverse when full or dispensing
        } else {
            1.0   // Forward to intake
        }
    }

    private suspend fun handleDetections() {
        if (isDispensing) return

        val r = colorSensor.red()
        val g = colorSensor.green()
        val b = colorSensor.blue()

        // No ball detected
        if (g <= 80 && b <= 110) {
            isSeen = false
            return
        }

        // Cooldown check
        val now = System.currentTimeMillis()
        if (now < Timing.nextDetectAllowedMs) return

        if (!isSeen) {
            val slot = nextSlot()
            if (slot != -1) {
                // Determine ball color: G = green/yellow, P = purple/other
                ord[slot] = if (g >= 110) "G" else "P"
                delay(200)
                LimelightUtil.advanceBowl(slot)
                Timing.nextDetectAllowedMs = now + Timing.DETECTION_COOLDOWN
            }
            isSeen = true
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

    private fun initializeHardware() {
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        intakeServo1 = hardwareMap.get(CRServo::class.java, "intakeServo1")
        colorSensor = hardwareMap.get(ColorSensor::class.java, "ColorSensor")

        setupMotorDirections()
        setupPIDFCoefficients()
        resetEncoders()
    }

    private fun setupMotorDirections() {
        outTake2.direction = DcMotorSimple.Direction.REVERSE
    }

    private fun setupPIDFCoefficients() {
        listOf(outTake1, outTake2).forEach {
            it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF)
        }
    }

    private fun resetEncoders() {
        listOf(outTake1, outTake2).forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    private fun initializePedroPathing() {
        panels = PanelsTelemetry.telemetry

        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()

        val otos = hardwareMap.get(SparkFunOTOS::class.java, "otos")
        otos.calibrateImu()
        otos.resetTracking()

        // Preload balls
        ord = arrayOf("P", "G", "P")

        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()

        buildPaths()
    }

    @JvmName("SetPathStateFunction")
    private fun setPathState(pState: Int) {
        pathState = pState
        pathTimer.resetTimer()
        timerState = false
    }
}