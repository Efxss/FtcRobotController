package org.firstinspires.ftc.teamcode

import android.util.Size
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.concurrent.Volatile
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

@Configurable
@TeleOp
class OutTake : OpMode() {
    private var panels: TelemetryManager? = null
    private lateinit var motor1: DcMotorEx
    private lateinit var motor2: DcMotorEx
    private var visionPortal: VisionPortal? = null
    private var tagProcessor: AprilTagProcessor? = null
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
    //endregion
    @Volatile
    var velocityModeInitialized = false
    var velocityPowerScale = 1.0
    companion object {
        @JvmField
        var power1 = 0.toDouble()
        var power2 = 0.toDouble()
        var p = 8.5.toDouble()
        var i = 0.05.toDouble()
        var d = 8.0.toDouble()
        var f = 11.7.toDouble()
    }

    override fun init() {
        motor1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        motor2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        motor2.direction = DcMotorSimple.Direction.REVERSE
        panels = PanelsTelemetry.telemetry
        f = 32767/motor1.motorType.achieveableMaxTicksPerSecond
        setupVision()
    }

    override fun loop() {
        motor1.setVelocityPIDFCoefficients(p, i, d, f)
        motor2.setVelocityPIDFCoefficients(p, i, d, f)
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
        val tagHeightPx = hypot(
            target.corners[3].x - target.corners[0].x,
            target.corners[3].y - target.corners[0].y
        )
        setMotorVelocityFromPseudoPower(motor1, power1) // 0.33
        setMotorVelocityFromPseudoPower(motor2, power2) // 0.33
        panels?.addData("Power1", power1)
        panels?.addData("Power2", power2)
        panels?.addData("Real Motor 1 Power", motor1.power)
        panels?.addData("Real Motor 2 Power", motor2.power)
        panels?.addData("Real Motor 1 velocity", motor1.velocity)
        panels?.addData("Real Motor 2 velocity", motor2.velocity)
        panels?.debug("H", tagHeightPx)
        panels?.update(telemetry)
    }

    private fun setupVision() {
        tagProcessor = AprilTagProcessor.easyCreateWithDefaults()
        visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessor(tagProcessor)
            .setCameraResolution(Size(1280, 720))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()
    }

    private fun ensureVelocityMode() {
        if (!velocityModeInitialized) {
            motor1.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor2.mode = DcMotor.RunMode.RUN_USING_ENCODER
            velocityModeInitialized = true
        }
    }

    private fun powerToTicksPerSecond(motor: DcMotorEx, power: Double): Double {
        val clipped = max(-1.0, min(1.0, power))
        val maxRpm = motor.motorType.maxRPM // no-load
        val tpr = motor.motorType.ticksPerRev
        val maxTicksPerSec = (maxRpm * tpr) / 60.0
        return clipped * velocityPowerScale * maxTicksPerSec
    }

    private fun setMotorVelocityFromPseudoPower(motor: DcMotorEx, power: Double) {
        ensureVelocityMode()
        val tps = powerToTicksPerSecond(motor, power)
        motor.velocity = tps
    }
}