package org.firstinspires.ftc.teamcode

import android.util.Size
import com.bylazar.configurables.annotations.IgnoreConfigurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.util.Timer
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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
import kotlin.concurrent.thread
import kotlin.math.max
import kotlin.math.min

@TeleOp
class TeleOP : OpMode() {

    @IgnoreConfigurable
    var panels: TelemetryManager? = null

    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer
    private val slowMode = false
    private val slowModeMultiplier = 0.5

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

    private var dispensingState = 0
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
        const val CENTER_DEADZONE = 15
        const val KP_ROTATE = 0.003
        const val OUTTAKE_SPEED = 0.26
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
    }

    override fun loop() {
        follower.update()
        handleDriving()
        runTelemetry()
    }

    private fun handleDriving() {
        thread {
            val rotate = (gamepad1.left_trigger - gamepad1.right_trigger)
            follower.setTeleOpDrive(
                gamepad1.left_stick_y.toDouble(),
                -gamepad1.right_stick_x.toDouble(),
                -rotate.toDouble(),
                false
            )
        }
    }

    private fun runTelemetry() {
        thread {
            val pose = follower.pose
            panels?.addData("EORD", expectedOrder)
            panels?.addData("Heading", pose.heading)
            panels?.addData("X", pose.x)
            panels?.addData("Y", pose.y)
            panels?.update(telemetry)
        }
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
        listOf(intakeServo2, outTake2)
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
    }

    private fun initializePedroPathing() {
        panels = PanelsTelemetry.telemetry

        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()

        follower = Constants.createFollower(hardwareMap)
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

    private fun read(): Int {
        val file = File(hardwareMap.appContext.filesDir, AprilTagIds.FILENAME)
        return (if (file.exists()) {
            file.readText()
        } else {
            0
        }) as Int
    }

    fun getOrder() {
        var tagID = read()
        when (tagID) {
            21 -> {
                expectedOrder.slot1 = PieceColor.GREEN
                expectedOrder.slot2 = PieceColor.PURPLE
                expectedOrder.slot3 = PieceColor.PURPLE
            }
            22 -> {
                expectedOrder.slot1 = PieceColor.PURPLE
                expectedOrder.slot2 = PieceColor.GREEN
                expectedOrder.slot3 = PieceColor.PURPLE
            }
            23 -> {
                expectedOrder.slot1 = PieceColor.PURPLE
                expectedOrder.slot2 = PieceColor.PURPLE
                expectedOrder.slot3 = PieceColor.GREEN
            }
        }
    }
}