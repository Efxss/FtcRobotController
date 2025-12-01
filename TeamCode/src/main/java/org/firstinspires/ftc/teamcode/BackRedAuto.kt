package org.firstinspires.ftc.teamcode

import android.util.Size
import com.bylazar.configurables.annotations.IgnoreConfigurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.io.File
import java.lang.Thread.sleep
import kotlin.concurrent.thread
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

@Autonomous(name = "Back Red Auto", group = "Main Red")
class BackRedAuto : OpMode() {
    @IgnoreConfigurable
    var panels: TelemetryManager? = null
    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer
    private lateinit var outTake1: DcMotorEx
    private lateinit var outTake2: DcMotorEx
    private lateinit var intakeServo1: CRServo
    private lateinit var intakeServo2: CRServo
    private lateinit var bowlServo: Servo
    private lateinit var camServo: Servo
    private lateinit var limelight: Limelight3A
    private var visionPortal: VisionPortal? = null
    private var tagProcessor: AprilTagProcessor? = null
    private var pathState: Int = 0
    private val startPose    = Pose(72.0, 0.0, Math.toRadians(90.0))
    private val preloadPose  = Pose(74.0, 4.0, Math.toRadians(80.0))
    private val pickupPoint5 = Pose(80.0, 17.5, Math.toRadians(9.0))
    private val pickup1      = Pose(86.5, 24.0, Math.toRadians(0.0))
    private val pickup1Ball1 = Pose(93.3, 24.0, Math.toRadians(0.0))
    private val pickup1Ball2 = Pose(96.3, 24.0, Math.toRadians(0.0))
    private val pickup1Ball3 = Pose(104.8, 24.0, Math.toRadians(0.0))
    private val scoreBack    = Pose(74.0, 10.0, Math.toRadians(73.0))
    private lateinit var preloadPose1: PathChain
    private lateinit var pickupPosePoint5: PathChain
    private lateinit var pickupPose1: PathChain
    private lateinit var pickupPose1Ball1: PathChain
    private lateinit var pickupPose1Ball2: PathChain
    private lateinit var pickupPose1Ball3: PathChain
    private lateinit var returnPose: PathChain
    private var dispensingState = 0
    private val pidP = 8.5
    private val pidI = 0.05
    private val pidD = 8.0
    private var pidF = 11.7
    private var velocityModeInitialized = false
    private var velocityPowerScale = 1.0
    private var intake = 0
    object ServoPositions {
        // Loading positions
        const val LOAD_P1 = 0.0
        const val LOAD_P2 = 0.075
        const val LOAD_P3 = 0.148

        // Firing/dispensing positions
        const val FIRE_P1 = 0.114
        const val FIRE_P2 = 0.1845
        const val FIRE_P3 = 0.258

        // Camera servo positions
        const val CAM_OPEN = 0.5
        const val CAM_CLOSED = 0.255
    }
    object DetectionThresholds {
        const val MIN_WIDTH = 200.0
        const val MIN_HEIGHT = 90.0
        const val MIN_Y_POSITION = 0.44
    }
    object Timing {
        const val DISPENSE_INITIAL_DELAY = 3000L
        const val BOWL_MOVE_DELAY = 1300L
        const val CAM_OPEN_DELAY = 500L
        const val CAM_CLOSE_DELAY = 2000L
        const val DETECTION_COOLDOWN = 1300L
        const val OUTTAKE_DELAY = 1000L
    }
    object ColorIds {
        const val GREEN = 1
        const val PURPLE = 2
    }
    object AprilTagIds {
        const val FILENAME = "tower.txt"
        const val GPP_ORDER = 21
        const val PGP_ORDER = 22
        const val PPG_ORDER = 23
        const val RED_DEPO =  24
    }
    object DepoCenter {
        const val DESIRED_TAG_WIDTH_PX = 110
        const val ROTATE_POWER = 0.2
        const val CAM_WIDTH_PX = 1280
        const val CAM_HEIGHT_PX = 720
        const val CENTER_DEADZONE = 16
        const val KP_ROTATE = 0.003
        const val OUTTAKE_SPEED = 0.27
    }
    private enum class PieceColor(val symbol: String) {
        NONE("N"),
        GREEN("G"),
        PURPLE("P");

        companion object {
            fun fromString(s: String): PieceColor =
                entries.find { it.symbol == s } ?: NONE  // Changed from values() to entries
        }
    }
    private data class GamePieceOrder(
        var slot1: PieceColor = PieceColor.NONE,
        var slot2: PieceColor = PieceColor.NONE,
        var slot3: PieceColor = PieceColor.NONE
    ) {
        fun isFull() = slot1 != PieceColor.NONE &&
                slot2 != PieceColor.NONE &&
                slot3 != PieceColor.NONE

        fun isEmpty() = slot1 == PieceColor.NONE &&
                slot2 == PieceColor.NONE &&
                slot3 == PieceColor.NONE

        fun contains(color: PieceColor) = slot1 == color || slot2 == color || slot3 == color

        fun addPiece(color: PieceColor): Boolean {
            //if (contains(color)) return false

            when {
                slot1 == PieceColor.NONE -> { slot1 = color; return true }
                slot2 == PieceColor.NONE -> { slot2 = color; return true }
                slot3 == PieceColor.NONE -> { slot3 = color; return true }
            }
            return false
        }

        fun reset() {
            slot1 = PieceColor.NONE
            slot2 = PieceColor.NONE
            slot3 = PieceColor.NONE
        }

        fun matches(expected: GamePieceOrder) =
            slot1 == expected.slot1 && slot2 == expected.slot2 && slot3 == expected.slot3

        override fun toString() = "${slot1.symbol}${slot2.symbol}${slot3.symbol}"
    }
    data class Target(
        val tx: Double,
        val ty: Double,
        val ta: Double,
        val colorId: Int,
        val width: Double,
        val height: Double
    ) {
        fun meetsDetectionThreshold() =
            width >= DetectionThresholds.MIN_WIDTH &&
                    height >= DetectionThresholds.MIN_HEIGHT &&
                    ty > DetectionThresholds.MIN_Y_POSITION
    }
    private val currentOrder = GamePieceOrder()
    private val expectedOrder = GamePieceOrder()
    private var currentLoadPosition = 1
    override fun init() {
        initializeHardware()
        initializePedroPathing()
    }

    override fun start() {
        limelight.start()
        opmodeTimer.resetTimer()
        setPathState(0)
        runOutTake()
        runTelemetryThread()
    }

    override fun loop() {
        follower.update()
        processAprilTags()
        autonomousPathUpdate()
        handleIntake()
        processVisionDetection()

        val pose = follower.pose

        // Optional numeric telemetry to Panels
        /*panels?.debug("path state", pathState)
        panels?.debug("x", pose.x)
        panels?.debug("y", pose.y)
        panels?.debug("heading", Math.toDegrees(pose.heading))
        panels?.debug("pathTimer", pathTimer.elapsedTimeSeconds)
        panels?.debug("actionTimer", actionTimer.elapsedTimeSeconds)
        panels?.debug("opmodeTimer", opmodeTimer.elapsedTimeSeconds)
        panels?.debug("Heading error", follower.headingError)
        panels?.debug("EORD", expectedOrder)
        panels?.debug("ORD", currentOrder)
        panels?.debug("CLP", currentLoadPosition)
        panels?.update(telemetry)*/
    }
    @Volatile
    private var isRunning = false
    private fun runTelemetryThread() {
        isRunning = true
        thread {
            while (isRunning) {
                try {
                    panels?.debug("OutTake1Power", outTake1.power)
                    panels?.debug("OutTake2Power", outTake2.power)
                    panels?.debug("OutTake1velocity", outTake1.velocity)
                    panels?.debug("OutTake2velocity", outTake2.velocity)
                    panels?.update(telemetry)
                } catch (e: Exception) {
                    break
                }
                sleep(50)
            }
        }
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
    }

    private fun autonomousPathUpdate() {
        when (pathState) {
            0 -> {
                follower.followPath(preloadPose1, true)
                setPathState(1)
            }
            1 -> {
                if (!follower.isBusy) {
                    setPathState(2)
                }
            }
            2 -> {
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    setPathState(3)
                }
            }
            3 -> {
                centerDepo()
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    setPathState(4)
                }
            }
            4 -> {
                dispensingState = 1
                handleDispensingStateMachine()
                if (dispensingState == 1) {
                    /* Do nothing shooting */
                } else {
                    setPathState(5)
                    sleep(250)
                }
            }
            5 -> {
                if (!follower.isBusy/* && pathTimer.elapsedTimeSeconds > 0.25 */) {
                    follower.followPath(pickupPosePoint5, false)
                    setPathState(6)
                    sleep(500)
                }
            }
            6 -> {
                if (!follower.isBusy/* && pathTimer.elapsedTimeSeconds > 0.5*/) {
                    follower.setMaxPower(0.4)
                    follower.followPath(pickupPose1, true)
                    setPathState(7)
                    sleep(500)
                }
            }
            7 -> {
                if (!follower.isBusy/* && pathTimer.elapsedTimeSeconds > 0.5*/) {
                    follower.setMaxPower(0.2)
                    intake = 1
                    follower.followPath(pickupPose1Ball1, true)
                    setPathState(8)
                    sleep(500)
                }
            }
            8 -> {
                if (!follower.isBusy/* && pathTimer.elapsedTimeSeconds > 0.5*/) {
                    follower.followPath(pickupPose1Ball2, true)
                    setPathState(9)
                    sleep(1000)
                }
            }
            9 -> {
                if (!follower.isBusy/* && pathTimer.elapsedTimeSeconds > 1.0*/) {
                    follower.followPath(pickupPose1Ball3, true)
                    setPathState(10)
                    sleep(1500)
                }
            }
            10 -> {
                if (!follower.isBusy/* && pathTimer.elapsedTimeSeconds > 1.5*/) {
                    follower.setMaxPower(0.6)
                    follower.followPath(returnPose, true)
                    setPathState(11)
                    sleep(2000)
                }
            }
            11 -> {
                if (!follower.isBusy/* && pathTimer.elapsedTimeSeconds > 2.0 */) {
                    setPathState(12)
                }
            }
            12 -> {
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    setPathState(13)
                }
            }
            13 -> {
                intake = 0
                centerDepo()
                if (pathTimer.elapsedTimeSeconds > 0.5) {
                    setPathState(14)
                }
            }
            14 -> {
                dispensingState = 1
                handleDispensingStateMachine()
            }
        }
    }

    private fun handleIntake() {
        when (intake) {
            0 -> {
                intakeServo1.power = 0.0
                intakeServo2.power = 0.0
            }
            1 -> {
                intakeServo1.power = 1.0
                intakeServo2.power = 1.0
            }
        }
    }
    private fun parsePythonOutput(py: DoubleArray): List<Target> {
        val stride = 6
        val targets = ArrayList<Target>()
        var i = 0
        while (i + stride - 1 < py.size) {
            targets.add(
                Target(
                    tx = py[i],
                    ty = py[i + 1],
                    ta = py[i + 2],
                    colorId = py[i + 3].toInt(),
                    width = py[i + 4],
                    height = py[i + 5]
                )
            )
            i += stride
        }
        return targets
    }
    private fun processVisionDetection() {
        val result = limelight.latestResult ?: return

        //panels?.debug("Data age (ms)", result.staleness)

        val py = result.pythonOutput
        if (py == null || py.isEmpty()) {
            //panels?.addLine("Python output empty")
            return
        }

        val targets = parsePythonOutput(py)
        val greenCount = targets.count { it.colorId == ColorIds.GREEN }
        val purpleCount = targets.count { it.colorId == ColorIds.PURPLE }

        /*panels?.debug("Targets", targets.size)
        panels?.debug("Green", greenCount)
        panels?.debug("Purple", purpleCount)*/

        // Process best target
        targets.maxByOrNull { it.ta }?.let { best ->
            if (best.meetsDetectionThreshold()) {
                when (best.colorId) {
                    ColorIds.GREEN -> attemptAddPiece(PieceColor.GREEN)
                    ColorIds.PURPLE -> attemptAddPiece(PieceColor.PURPLE)
                }

                // Display best target info
                /*panels?.addLine("Best (by area)")
                panels?.debug(" colorId", best.colorId)
                panels?.debug(" tx(norm)", best.tx)
                panels?.debug(" ty(norm)", best.ty)
                panels?.debug(" area(px)", best.ta)
                panels?.debug(" W", best.width)
                panels?.debug(" H", best.height)
                panels?.update(telemetry)*/
            }
        }
    }
    private fun attemptAddPiece(color: PieceColor) {
        if (currentOrder.isFull()) {
            dispensingState = 1
            return
        }

        if (currentOrder.addPiece(color)) {
            advanceBowlPosition()
            sleep(Timing.DETECTION_COOLDOWN)
        }
    }
    private fun advanceBowlPosition() {
        when (currentLoadPosition) {
            1 -> {
                bowlServo.position = ServoPositions.LOAD_P2
                currentLoadPosition = 2
            }
            2 -> {
                bowlServo.position = ServoPositions.LOAD_P3
                currentLoadPosition = 3
            }
            3 -> {
                bowlServo.position = ServoPositions.FIRE_P2
                handleDispensingStateMachine()
            }
        }
    }
    private fun handleDispensingStateMachine() {
        when (dispensingState) {
            0 -> { /* Idle - collecting pieces */ }
            1 -> executeDispensing()
        }
    }
    private fun executeDispensing() {
        lockRobot()
        // Raise outtake
        //setMotorVelocityFromPseudoPower(outTake1, DepoCenter.OUTTAKE_SPEED)
        //setMotorVelocityFromPseudoPower(outTake2, DepoCenter.OUTTAKE_SPEED)
        sleep(Timing.OUTTAKE_DELAY)

        if (currentOrder.isFull() && !expectedOrder.isEmpty()) {
            val dispenseSequence = calculateDispenseSequence()
            if (dispenseSequence != null) {
                executeDispenseSequence(dispenseSequence)
            }
        }

        // Lower outtake and reset
        sleep(Timing.OUTTAKE_DELAY)
        bowlServo.position = ServoPositions.LOAD_P1
        sleep(Timing.OUTTAKE_DELAY)

        dispensingState = 0
        currentLoadPosition = 1
        currentOrder.reset()
        unlockRobot(0.6)
    }
    private fun calculateDispenseSequence(): List<Double>? {
        // Map: Expected pattern -> Current pattern -> Dispense sequence
        val sequenceMap = mapOf(
            "GPP" to mapOf(
                "GPP" to listOf(ServoPositions.FIRE_P1, ServoPositions.FIRE_P2, ServoPositions.FIRE_P3),
                "PPG" to listOf(ServoPositions.FIRE_P3, ServoPositions.FIRE_P2, ServoPositions.FIRE_P1),
                "PGP" to listOf(ServoPositions.FIRE_P2, ServoPositions.FIRE_P1, ServoPositions.FIRE_P3)
            ),
            "PGP" to mapOf(
                "PGP" to listOf(ServoPositions.FIRE_P1, ServoPositions.FIRE_P2, ServoPositions.FIRE_P3),
                "PPG" to listOf(ServoPositions.FIRE_P1, ServoPositions.FIRE_P3, ServoPositions.FIRE_P2),
                "GPP" to listOf(ServoPositions.FIRE_P2, ServoPositions.FIRE_P1, ServoPositions.FIRE_P3)
            ),
            "PPG" to mapOf(
                "PPG" to listOf(ServoPositions.FIRE_P1, ServoPositions.FIRE_P2, ServoPositions.FIRE_P3),
                "PGP" to listOf(ServoPositions.FIRE_P1, ServoPositions.FIRE_P3, ServoPositions.FIRE_P2),
                "GPP" to listOf(ServoPositions.FIRE_P2, ServoPositions.FIRE_P3, ServoPositions.FIRE_P1)
            )
        )

        return sequenceMap[expectedOrder.toString()]?.get(currentOrder.toString())
    }
    private fun executeDispenseSequence(positions: List<Double>) {
        sleep(Timing.DISPENSE_INITIAL_DELAY)

        positions.forEach { position ->
            bowlServo.position = position
            sleep(Timing.BOWL_MOVE_DELAY)

            // Trigger cam servo
            camServo.position = ServoPositions.CAM_OPEN
            sleep(Timing.CAM_OPEN_DELAY)
            camServo.position = ServoPositions.CAM_CLOSED
            sleep(Timing.CAM_CLOSE_DELAY)
        }
    }
    private fun setupVision() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults()
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(tagProcessor)
            .setCameraResolution(Size(1280, 720))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()
    }
    private fun processAprilTags() {
        val detections = tagProcessor?.detections.orEmpty()
        val tagIds = detections.map { it.id }

        tagIds.firstOrNull()?.let { id ->
            when (id) {
                AprilTagIds.GPP_ORDER -> {
                    expectedOrder.slot1 = PieceColor.GREEN
                    expectedOrder.slot2 = PieceColor.PURPLE
                    expectedOrder.slot3 = PieceColor.PURPLE
                    saveNumber(21)
                }
                AprilTagIds.PGP_ORDER -> {
                    expectedOrder.slot1 = PieceColor.PURPLE
                    expectedOrder.slot2 = PieceColor.GREEN
                    expectedOrder.slot3 = PieceColor.PURPLE
                    saveNumber(22)
                }
                AprilTagIds.PPG_ORDER -> {
                    expectedOrder.slot1 = PieceColor.PURPLE
                    expectedOrder.slot2 = PieceColor.PURPLE
                    expectedOrder.slot3 = PieceColor.GREEN
                    saveNumber(23)
                }
            }
        }
    }
    private fun centerDepo() {
        val detections = tagProcessor?.detections.orEmpty()
        val target = detections.firstOrNull { it.id == AprilTagIds.RED_DEPO }
        if (target == null) {
            return
        }
        val xErrPx: Double = target.center.x - (DepoCenter.CAM_WIDTH_PX / 2.0)
        val tagWidthPx = hypot(
            target.corners[1].x - target.corners[0].x,
            target.corners[1].y - target.corners[0].y
        )
        val widthErrPx = DepoCenter.DESIRED_TAG_WIDTH_PX - tagWidthPx
        if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
            if (pathState == 3) {
                setPathState(4)
            } else if (pathState == 13) {
                setPathState(14)
            }
            return
        }
        val hFovDeg = 70.0
        val hFovRad = Math.toRadians(hFovDeg)
        val pixelsToRad = hFovRad / DepoCenter.CAM_WIDTH_PX
        val angleError = xErrPx * pixelsToRad
        val maxTurnStepDeg = 10.0
        val maxTurnStepRad = Math.toRadians(maxTurnStepDeg)
        val turnStep = clip(
            angleError,
            -maxTurnStepRad,
            maxTurnStepRad
        )
        val currentHeading = follower.pose.heading
        val targetHeading = currentHeading - turnStep
        follower.turnTo(targetHeading)
    }
    private fun clip(v: Double, min: Double, max: Double): Double {
        return max(min, min(max, v))
    }
    private fun initializeHardware() {
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        intakeServo1 = hardwareMap.get(CRServo::class.java, "intakeServo1")
        intakeServo2 = hardwareMap.get(CRServo::class.java, "intakeServo2")
        bowlServo = hardwareMap.get(Servo::class.java, "bowlServo")
        camServo = hardwareMap.get(Servo::class.java, "camServo")
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        camServo.position = ServoPositions.CAM_CLOSED
        bowlServo.position = ServoPositions.LOAD_P1
        setupMotorDirections()
        setupPIDFCoefficients()
        resetEncoders()
        setupVision()
    }
    private fun setupMotorDirections() {
        listOf(intakeServo2, outTake2)
            .forEach { it.direction = DcMotorSimple.Direction.REVERSE }
    }
    private fun setupPIDFCoefficients() {
        pidF = 32767/outTake1.motorType.achieveableMaxTicksPerSecond
        listOf(outTake1, outTake2)
            .forEach { it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF) }
    }
    private fun resetEncoders() {
        listOf(outTake1, outTake2).forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }
    }
    private fun runOutTake() {
        listOf(outTake1, outTake2)
            .forEach { setMotorVelocityFromPseudoPower(it, DepoCenter.OUTTAKE_SPEED) }
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

        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()
        currentOrder.slot1 = PieceColor.PURPLE
        currentOrder.slot2 = PieceColor.GREEN
        currentOrder.slot3 = PieceColor.PURPLE
        buildPaths()
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
    private fun lockRobot() {
        follower.pausePathFollowing()
        follower.setMaxPower(0.0)
    }
    private fun unlockRobot(maxPower: Double) {
        follower.setMaxPower(maxPower)
        follower.resumePathFollowing()
    }
    private fun setPathState(pState: Int) {
        pathState = pState
        pathTimer.resetTimer()
    }
    private fun saveNumber(num: Int) {
        val file = File(hardwareMap.appContext.filesDir, AprilTagIds.FILENAME)
        file.writeText(num.toString())
    }
    override fun stop() {
        isRunning = false
    }
}
