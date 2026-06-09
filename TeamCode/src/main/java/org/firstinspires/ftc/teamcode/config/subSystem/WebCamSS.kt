package org.firstinspires.ftc.teamcode.config.subSystem

import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.config.util.Alliance
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs

class WebCamSS(
    hardwareMap : HardwareMap,
    webcamName : String = "Webcam 1"
) {
    private var aprilTag : AprilTagProcessor? = null
    private var visionPortal : VisionPortal? = null
    private val cameraPosition : Position = Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0)
    private val cameraOrientation = YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -90.0, 0.0, 0)
    private var cameraRes = Size(1280, 800)
    init { initAprilTag(hardwareMap, webcamName) }
    fun currentTagXRad(alliance : Alliance, deadzone : Double) : Double {
        val targetId = when (alliance) {
            Alliance.BLUE -> 20
            Alliance.RED -> 24
        }
        val detection = aprilTag?.detections?.firstOrNull { it.id == targetId && it.ftcPose != null } ?: return 0.0
        val tr = detection.ftcPose.bearing
        return if (tr in 0.0..abs(deadzone)) 0.0 else tr
    }
    fun isTagSeen(alliance: Alliance) : Boolean {
        val targetId = when (alliance) {
            Alliance.BLUE -> 20
            Alliance.RED -> 24
        }
        val detection = aprilTag?.detections?.firstOrNull { it.id == targetId && it.ftcPose != null } ?: return false
        return detection.metadata != null
    }
    private fun initAprilTag(hardwareMap : HardwareMap, webcamName : String) {
        aprilTag = AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build()
        val builder = VisionPortal.Builder()
        builder.setCamera(hardwareMap.get(WebcamName::class.java, webcamName))
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        builder.setCameraResolution(cameraRes)
        builder.addProcessor(aprilTag)
        visionPortal = builder.build()
    }
    fun stop() { visionPortal?.close() }
}