package org.firstinspires.ftc.teamcode

import android.util.Size
import com.bylazar.configurables.annotations.IgnoreConfigurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
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
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Thread.sleep
import kotlin.math.abs
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

@TeleOp
class RedTeleOP : OpMode() {
    @IgnoreConfigurable
    var panels: TelemetryManager? = null
    private val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    private var outTakeCalc: Job? = null
    private var manualFire: Job?  = null
    private var actVision: Job?   = null
    private var patDect: Job?     = null
    private var actLift: Job?     = null
    private val startPose = Pose(72.0, 72.0, Math.toRadians(0.0))
    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer
    private lateinit var outTake1:  DcMotorEx
    private lateinit var outTake2:  DcMotorEx
    private lateinit var liftLeft:  DcMotorEx
    private lateinit var liftRight: DcMotorEx
    private lateinit var intakeServo1: CRServo
    private lateinit var intakeServo2: CRServo
    private lateinit var bowlServo: Servo
    private lateinit var camServo: Servo
    private lateinit var limelight: Limelight3A
    private var visionPortal: VisionPortal? = null
    private var tagProcessor: AprilTagProcessor? = null
    private var pathState: Int = 0
    private var dispensingState = 0
    private var isCentering = false
    private var rightBumperPressed = false
    var endgameTogglePressed = false
    private val pidP = 8.05
    private val pidI = 0.6
    private val pidD = 0.9
    private val pidF = 0.01
    private var velocityModeInitialized = false
    private var velocityPowerScale = 0.95
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

        const val INTAKE_ON = 1.0
        const val INTAKE_REVERSE = -1.0
        const val INTAKE_OFF = 0.0
    }
    object DetectionThresholds {
        const val MIN_WIDTH = 200.0
        const val MIN_HEIGHT = 90.0
        const val MIN_Y_POSITION = 0.76
    }
    object Timing {
        const val DISPENSE_INITIAL_DELAY = 100L
        const val BOWL_MOVE_DELAY = 1300L
        const val CAM_OPEN_DELAY = 500L
        const val CAM_CLOSE_DELAY = 2000L
        const val DETECTION_COOLDOWN = 1500L
        const val OUTTAKE_DELAY = 1000L
    }
    object ColorIds {
        const val GREEN = 1
        const val PURPLE = 2
    }
    object AprilTagIds {
        const val GPP_ORDER = 21
        const val PGP_ORDER = 22
        const val PPG_ORDER = 23
        const val RED_DEPO =  24
    }
    object EndGame {
        const val LIFTMAX = 11400
        const val SLOWMODE = 1000
        const val NORMALSPEED = 0.6
        const val SLOWSPEED = 0.2
        var ISENDGAME = 0
    }
    object DepoCenter {
        const val DESIRED_TAG_WIDTH_PX = 110
        const val ROTATE_POWER = 0.2
        const val CAM_WIDTH_PX = 1280
        const val CAM_HEIGHT_PX = 720
        const val CENTER_DEADZONE = 15
        const val KP_ROTATE = 0.003
        var OUTTAKE_SPEED = 0.20
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
        follower.startTeleOpDrive()
        outTakeCalc = scope.launch {
            while (isActive) {
                outTakePower()
                delay(50)
            }
        }
        manualFire = scope.launch {
            while (isActive) {
                overrideShoot()
                delay(10)
            }
        }
        actVision = scope.launch {
            while (isActive) {
                processVisionDetection()
                delay(5)
            }
        }
        patDect = scope.launch {
            while (isActive) {
                processAprilTags()
                delay(5)
            }
        }
        actLift = scope.launch {
            while(isActive){
                runLift()
                delay(20)
            }
        }
    }

    override fun loop() {
        follower.update()

        if (gamepad1.right_bumper && !rightBumperPressed) {
            isCentering = !isCentering
            if (!isCentering) {
                follower.startTeleOpDrive()
            }
        }
        rightBumperPressed = gamepad1.right_bumper

        if (isCentering) {
            centerDepo()
        } else {
            follower.setMaxPower(0.6)
            val rotate = (gamepad1.left_trigger - gamepad1.right_trigger)
            follower.setTeleOpDrive(
                -gamepad1.left_stick_y.toDouble(),
                gamepad1.right_stick_x.toDouble(),
                rotate * 0.5,
                true
            )
        }
        val endgameButtonPressed = gamepad1.left_bumper && gamepad1.cross
        if (endgameButtonPressed && !endgameTogglePressed) {
            EndGame.ISENDGAME = if (EndGame.ISENDGAME == 0) 1 else 0
        }
        endgameTogglePressed = endgameButtonPressed

        handleIntake()
        /*panels?.debug("heading", follower.pose.heading)
        panels?.debug("X", follower.pose.x)
        panels?.debug("Y", follower.pose.y)
        panels?.debug("EORD", expectedOrder)
        panels?.debug("ORD", currentOrder)
        panels?.debug("Lift Left Power", liftLeft.power)
        panels?.debug("Lift Right Power", liftRight.power)
        panels?.debug("Lift Left Position", liftLeft.currentPosition)
        panels?.debug("Lift Right Position", liftRight.currentPosition)
        panels?.update(telemetry)*/
    }

    private fun runLift() {
        var isRunning = false
        when (EndGame.ISENDGAME) {
            0 -> { /* Do nothing */ }
            1 -> {
                if (gamepad1.dpad_up && gamepad1.triangle) {
                    isRunning = true
                    outTakeCalc?.cancel()
                    outTake1.power=0.0
                    outTake2.power=0.0
                }
                if (isRunning) {
                    if (liftLeft.currentPosition < EndGame.SLOWMODE || liftRight.currentPosition < EndGame.SLOWMODE) {
                        listOf(liftLeft, liftRight).forEach { motor ->
                            motor.power = EndGame.NORMALSPEED
                        }
                    } else if (liftLeft.currentPosition > EndGame.SLOWMODE || liftRight.currentPosition > EndGame.SLOWMODE) {
                        listOf(liftLeft, liftRight).forEach { motor ->
                            motor.power = EndGame.SLOWSPEED
                        }
                    } else if (liftLeft.currentPosition > EndGame.LIFTMAX || liftRight.currentPosition > EndGame.LIFTMAX) {
                        listOf(liftLeft, liftRight).forEach { motor ->
                            motor.power = 0.0
                        }
                    }
                }
            }
        }
    }
    private fun overrideShoot() {
        if (gamepad1.square) {
            outTakeCalc?.cancel()
            outTake1.power = DepoCenter.OUTTAKE_SPEED
            outTake2.power = DepoCenter.OUTTAKE_SPEED
            val randDispenseSequence = listOf(ServoPositions.FIRE_P3, ServoPositions.FIRE_P2, ServoPositions.FIRE_P1)
            executeDispenseSequence(randDispenseSequence)
            currentOrder.reset()
            bowlServo.position = ServoPositions.LOAD_P1
            outTakeCalc?.start()
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
            return
        }

        val targets = parsePythonOutput(py)
        val greenCount = targets.count { it.colorId == ColorIds.GREEN }
        val purpleCount = targets.count { it.colorId == ColorIds.PURPLE }

        // Process best target
        targets.maxByOrNull { it.ta }?.let { best ->
            if (best.meetsDetectionThreshold()) {
                when (best.colorId) {
                    ColorIds.GREEN -> attemptAddPiece(PieceColor.GREEN)
                    ColorIds.PURPLE -> attemptAddPiece(PieceColor.PURPLE)
                }
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
        sleep(Timing.OUTTAKE_DELAY)

        if (currentOrder.isFull() && !expectedOrder.isEmpty()) {
            when (EndGame.ISENDGAME) {
                0 -> {
                    val randDispenseSequence = listOf(ServoPositions.FIRE_P3, ServoPositions.FIRE_P2, ServoPositions.FIRE_P1)
                    executeDispenseSequence(randDispenseSequence)
                }
                1 -> {
                    val dispenseSequence = calculateDispenseSequence()
                    if (dispenseSequence != null) {
                        executeDispenseSequence(dispenseSequence)
                    }
                }
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
            camServo.position = ServoPositions.CAM_OPEN
            sleep(Timing.CAM_OPEN_DELAY)
            camServo.position = ServoPositions.CAM_CLOSED
            sleep(Timing.CAM_CLOSE_DELAY)
        }
    }
    private fun outTakePower() {
        val detections = tagProcessor?.detections.orEmpty()
        val target = detections.firstOrNull { it.id == AprilTagIds.RED_DEPO }
        if (target == null) {
            return
        }
        val tagHeightPx = hypot(
            target.corners[3].x - target.corners[0].x,
            target.corners[3].y - target.corners[0].y
        )
        val heightCalc = tagHeightPx-65
        val scale = heightCalc*0.000685
        val powerResult = 0.27-scale
        DepoCenter.OUTTAKE_SPEED = powerResult
        outTake1.power = DepoCenter.OUTTAKE_SPEED
        outTake2.power = DepoCenter.OUTTAKE_SPEED
    }
    private fun handleIntake() {
        if (gamepad1.cross) {
            intakeServo1.power = ServoPositions.INTAKE_ON
            intakeServo2.power = ServoPositions.INTAKE_ON
        } else if (gamepad1.circle) {
            intakeServo1.power = ServoPositions.INTAKE_REVERSE
            intakeServo2.power = ServoPositions.INTAKE_REVERSE
        }
        else {
            intakeServo1.power = ServoPositions.INTAKE_OFF
            intakeServo2.power = ServoPositions.INTAKE_OFF
        }
    }
    private fun centerDepo() {
        follower.setMaxPower(0.1)
        val detections = tagProcessor?.detections.orEmpty()
        val target = detections.firstOrNull { it.id == AprilTagIds.RED_DEPO }

        if (target == null) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
            return
        }

        val xErrPx: Double = target.center.x - (DepoCenter.CAM_WIDTH_PX / 2.0)

        if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)

            dispensingState = 1
            executeDispensing()

            isCentering = false
            unlockRobot(0.6)

            return
        }

        val rotationPower = clip(xErrPx * DepoCenter.KP_ROTATE, -0.3, 0.3)

        follower.setTeleOpDrive(0.0, 0.0, -rotationPower, false)
    }
    private fun initializeHardware() {
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        liftLeft = hardwareMap.get(DcMotorEx::class.java, "liftLeft")
        liftRight = hardwareMap.get(DcMotorEx::class.java, "liftRight")
        intakeServo1 = hardwareMap.get(CRServo::class.java, "intakeServo1")
        intakeServo2 = hardwareMap.get(CRServo::class.java, "intakeServo2")
        bowlServo = hardwareMap.get(Servo::class.java, "bowlServo")
        camServo = hardwareMap.get(Servo::class.java, "camServo")
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        camServo.position = ServoPositions.CAM_CLOSED
        bowlServo.position = ServoPositions.FIRE_P3
        setupMotorDirections()
        setupPIDFCoefficients()
        resetEncoders()
        setupVision()
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
    private fun setupMotorDirections() {
        listOf(intakeServo2, outTake2, liftLeft)
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
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }
        listOf(liftLeft, liftRight).forEach { motor ->
            motor.targetPosition = EndGame.LIFTMAX
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


        currentOrder.slot1 = PieceColor.PURPLE
        currentOrder.slot2 = PieceColor.GREEN
        currentOrder.slot3 = PieceColor.PURPLE

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
    private fun lockRobot() {
        follower.pausePathFollowing()
        follower.setMaxPower(0.0)
    }
    private fun unlockRobot(maxPower: Double) {
        follower.setMaxPower(maxPower)
        follower.resumePathFollowing()
        follower.startTeleOpDrive()
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
                    patDect?.cancel()
                }
                AprilTagIds.PGP_ORDER -> {
                    expectedOrder.slot1 = PieceColor.PURPLE
                    expectedOrder.slot2 = PieceColor.GREEN
                    expectedOrder.slot3 = PieceColor.PURPLE
                    patDect?.cancel()
                }
                AprilTagIds.PPG_ORDER -> {
                    expectedOrder.slot1 = PieceColor.PURPLE
                    expectedOrder.slot2 = PieceColor.PURPLE
                    expectedOrder.slot3 = PieceColor.GREEN
                    patDect?.cancel()
                }
            }
        }
    }
}
