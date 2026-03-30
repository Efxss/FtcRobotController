package org.firstinspires.ftc.teamcode.util

import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

class CenterUtil (
    hardwareMap : HardwareMap,
    private val rotatePower : Double = 0.2,
    private val deadzone : Int = 15,
    private val id : Int = 20
) {
    private var rotationPower = 0.0f
    private val camWidthPx = 1280
    private val camHeightPx = 720
    private val kpRotate = 0.003
    private var isCentering = false


    private val driveUtil = DriveUtil(
        hardwareMap,
        drivePower = rotatePower,
        deadzone = 0.2f
    )

    private val tagProcessor: AprilTagProcessor = AprilTagProcessor.Builder()
        .build()
    private val visionPortal: VisionPortal = VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        .setCameraResolution(Size(camWidthPx, camHeightPx))
        .addProcessor(tagProcessor)
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .build()

    fun centering(button: Boolean) {
        if (button) {
            centerDepo()
        }
    }

    fun centerDepo() {
        isCentering = true
        val detections = tagProcessor.detections
        val target = detections.firstOrNull { it.id == id }

        if (target == null) {
            stopDrive()
            return
        }

        val xErrPx = target.center.x - (camWidthPx / 2.0)

        if (abs(xErrPx) <= deadzone) {
            stopDrive()
            return
        }

        rotationPower = clip(xErrPx * kpRotate, -rotatePower, rotatePower).toFloat()
        driveUtil.tankDrive(-rotationPower, rotationPower)
    }

    private fun stopDrive() {
        driveUtil.tankDrive(0f, 0f)
        isCentering = false
    }

    private fun clip(v: Double, minValue: Double, maxValue: Double) : Double {
        return max(minValue, min(maxValue, v))
    }

    fun isCentering(): Boolean {
        return isCentering
    }

    fun getPower(): Float {
        return rotationPower
    }

    fun close() {
        visionPortal.close()
    }
}