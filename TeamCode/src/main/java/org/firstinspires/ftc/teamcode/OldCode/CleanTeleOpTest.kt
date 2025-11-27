package org.firstinspires.ftc.teamcode.OldCode

import android.util.Size
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Thread.sleep
import kotlin.math.max
import kotlin.math.min

@Disabled
class CleanTeleOpTest : OpMode() {
    // ========== HARDWARE ==========
    private var panels: TelemetryManager? = null
    private lateinit var frontLeft: DcMotorEx
    private lateinit var frontRight: DcMotorEx
    private lateinit var backLeft: DcMotorEx
    private lateinit var backRight: DcMotorEx
    private lateinit var outTake1: DcMotorEx
    private lateinit var outTake2: DcMotorEx
    private lateinit var intakeServo1: CRServo
    private lateinit var intakeServo2: CRServo
    private lateinit var bowlServo: Servo
    private lateinit var camServo: Servo
    private lateinit var limelight: Limelight3A
    private var visionPortal: VisionPortal? = null
    private var tagProcessor: AprilTagProcessor? = null

    // ========== CONSTANTS ==========
    object ServoPositions {
        // Loading positions
        const val LOAD_P1 = 0.059
        const val LOAD_P2 = 0.13
        const val LOAD_P3 = 0.204

        // Firing/dispensing positions
        const val FIRE_P1 = 0.167
        const val FIRE_P2 = 0.02
        const val FIRE_P3 = 0.0945

        // Camera servo positions
        const val CAM_OPEN = 0.5
        const val CAM_CLOSED = 0.0
    }

    object DetectionThresholds {
        const val MIN_WIDTH = 250.0
        const val MIN_HEIGHT = 110.0
        const val MIN_Y_POSITION = 0.55
    }

    object Timing {
        const val DISPENSE_INITIAL_DELAY = 5000L
        const val BOWL_MOVE_DELAY = 1500L
        const val CAM_OPEN_DELAY = 500L
        const val CAM_CLOSE_DELAY = 5000L
        const val DETECTION_COOLDOWN = 800L
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
    }

    // ========== STATE VARIABLES ==========
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
            if (contains(color)) return false

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

    private val currentOrder = GamePieceOrder()
    private val expectedOrder = GamePieceOrder()
    private var currentLoadPosition = 1
    private var driveSpeed = false
    private var dispensingState = 0

    // PIDF coefficients
    private val pidP = 10.0
    private val pidI = 3.0
    private val pidD = 0.0
    private val pidF = 8.0
    private var velocityModeInitialized = false
    private var velocityPowerScale = 0.85

    // ========== DATA CLASSES ==========
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

    // ========== INITIALIZATION ==========
    override fun init() {
        initializeHardware()
        setupMotorDirections()
        setupPIDFCoefficients()
        setupVision()
        panels = PanelsTelemetry.telemetry
        resetEncoders()
    }

    private fun initializeHardware() {
        frontLeft = hardwareMap.get(DcMotorEx::class.java, "frontLeft")
        frontRight = hardwareMap.get(DcMotorEx::class.java, "frontRight")
        backLeft = hardwareMap.get(DcMotorEx::class.java, "backLeft")
        backRight = hardwareMap.get(DcMotorEx::class.java, "backRight")
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        intakeServo1 = hardwareMap.get(CRServo::class.java, "intakeServo1")
        intakeServo2 = hardwareMap.get(CRServo::class.java, "intakeServo2")
        bowlServo = hardwareMap.get(Servo::class.java, "bowlServo")
        camServo = hardwareMap.get(Servo::class.java, "camServo")
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
    }

    private fun setupMotorDirections() {
        listOf(frontRight, frontLeft, intakeServo2, outTake2)
            .forEach { it.direction = DcMotorSimple.Direction.REVERSE }
        listOf(backRight, backLeft)
            .forEach { it.direction = DcMotorSimple.Direction.FORWARD }
    }

    private fun setupPIDFCoefficients() {
        listOf(frontRight, backRight, frontLeft, backLeft, outTake1, outTake2)
            .forEach { it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF) }
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

        panels?.addLine("Limelight init complete.")
    }

    override fun start() {
        limelight.start()
        bowlServo.position = ServoPositions.LOAD_P1
        camServo.position = ServoPositions.CAM_CLOSED
    }

    // ========== MAIN LOOP ==========
    override fun loop() {
        processAprilTags()
        handleDriveControls()
        handleIntakeControls()
        processVisionDetection()
        handleDispensingStateMachine()
        updateTelemetry()
    }

    // ========== APRILTAG PROCESSING ==========
    private fun processAprilTags() {
        val detections = tagProcessor?.detections.orEmpty()
        val tagIds = detections.map { it.id }

        tagIds.firstOrNull()?.let { id ->
            when (id) {
                AprilTagIds.GPP_ORDER -> {
                    expectedOrder.slot1 = PieceColor.GREEN
                    expectedOrder.slot2 = PieceColor.PURPLE
                    expectedOrder.slot3 = PieceColor.PURPLE
                    panels?.addLine("Detected Tag 21: GPP")
                }
                AprilTagIds.PGP_ORDER -> {
                    expectedOrder.slot1 = PieceColor.PURPLE
                    expectedOrder.slot2 = PieceColor.GREEN
                    expectedOrder.slot3 = PieceColor.PURPLE
                    panels?.addLine("Detected Tag 22: PGP")
                }
                AprilTagIds.PPG_ORDER -> {
                    expectedOrder.slot1 = PieceColor.PURPLE
                    expectedOrder.slot2 = PieceColor.PURPLE
                    expectedOrder.slot3 = PieceColor.GREEN
                    panels?.addLine("Detected Tag 23: PPG")
                }
            }
        }
    }

    // ========== DRIVE CONTROLS ==========
    private fun handleDriveControls() {
        val drive = gamepad1.left_stick_y.toDouble()
        val strafe = -gamepad1.right_stick_x.toDouble()
        val rotate = (gamepad1.left_trigger - gamepad1.right_trigger).toDouble()

        val frontLeftPower = drive + strafe + rotate
        val frontRightPower = drive - strafe - rotate
        val backLeftPower = drive - strafe + rotate
        val backRightPower = drive + strafe - rotate

        // Speed toggle
        if (gamepad1.left_bumper) driveSpeed = false
        else if (gamepad1.right_bumper) driveSpeed = true

        val speedMultiplier = if (driveSpeed) 8000.0 else 1000.0

        frontLeft.velocity = -frontLeftPower * speedMultiplier
        frontRight.velocity = frontRightPower * speedMultiplier
        backLeft.velocity = backLeftPower * speedMultiplier
        backRight.velocity = -backRightPower * speedMultiplier
    }

    // ========== INTAKE CONTROLS ==========
    private fun handleIntakeControls() {
        val servoSpeed = when {
            gamepad1.cross -> 1.0
            gamepad1.circle -> -1.0
            else -> 0.0
        }

        intakeServo1.power = servoSpeed
        intakeServo2.power = servoSpeed
    }

    // ========== VISION DETECTION ==========
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

        panels?.addData("Data age (ms)", result.staleness)

        val py = result.pythonOutput
        if (py == null || py.isEmpty()) {
            panels?.addLine("Python output empty")
            return
        }

        val targets = parsePythonOutput(py)
        val greenCount = targets.count { it.colorId == ColorIds.GREEN }
        val purpleCount = targets.count { it.colorId == ColorIds.PURPLE }

        panels?.addData("Targets", targets.size)
        panels?.addData("Green", greenCount)
        panels?.addData("Purple", purpleCount)

        // Process best target
        targets.maxByOrNull { it.ta }?.let { best ->
            if (best.meetsDetectionThreshold()) {
                when (best.colorId) {
                    ColorIds.GREEN -> attemptAddPiece(PieceColor.GREEN)
                    ColorIds.PURPLE -> attemptAddPiece(PieceColor.PURPLE)
                }

                // Display best target info
                panels?.addLine("Best (by area)")
                panels?.addData(" colorId", best.colorId)
                panels?.addData(" tx(norm)", best.tx)
                panels?.addData(" ty(norm)", best.ty)
                panels?.addData(" area(px)", best.ta)
                panels?.addData(" W", best.width)
                panels?.addData(" H", best.height)
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
            }
        }
    }

    // ========== DISPENSING STATE MACHINE ==========
    private fun handleDispensingStateMachine() {
        when (dispensingState) {
            0 -> { /* Idle - collecting pieces */ }
            1 -> executeDispensing()
        }
    }

    private fun executeDispensing() {
        // Raise outtake
        setMotorVelocityFromPseudoPower(outTake1, 0.2)
        setMotorVelocityFromPseudoPower(outTake2, 0.2)
        sleep(Timing.OUTTAKE_DELAY)

        if (currentOrder.isFull() && !expectedOrder.isEmpty()) {
            val dispenseSequence = calculateDispenseSequence()
            if (dispenseSequence != null) {
                executeDispenseSequence(dispenseSequence)
            }
        }

        // Lower outtake and reset
        sleep(Timing.OUTTAKE_DELAY)
        setMotorVelocityFromPseudoPower(outTake1, 0.0)
        setMotorVelocityFromPseudoPower(outTake2, 0.0)
        bowlServo.position = ServoPositions.LOAD_P1
        sleep(Timing.OUTTAKE_DELAY)

        dispensingState = 0
        currentLoadPosition = 1
        currentOrder.reset()
        expectedOrder.reset()
    }

    // ========== DISPENSING LOGIC ==========
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

    // ========== MOTOR VELOCITY HELPERS ==========
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

    // ========== ENCODER MANAGEMENT ==========
    private fun resetEncoders() {
        listOf(frontLeft, frontRight, backLeft, backRight).forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    // ========== TELEMETRY ==========
    private fun updateTelemetry() {
        panels?.addData("Current Order", currentOrder.toString())
        panels?.addData("Expected Order", expectedOrder.toString())
        panels?.addData("Load Position", currentLoadPosition)
        panels?.addData("Dispensing State", dispensingState)
        panels?.addData("Bowl Position", bowlServo.position)
        panels?.addData("Drive Speed", if (driveSpeed) "FAST" else "SLOW")
        panels?.update()
    }

    override fun stop() {
        limelight.stop()
        panels?.addData("Status", "Stopped")
        panels?.update()
    }
}