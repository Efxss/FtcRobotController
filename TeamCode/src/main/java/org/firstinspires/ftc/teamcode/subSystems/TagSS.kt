package org.firstinspires.ftc.teamcode.subSystems

import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Position
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles
import org.firstinspires.ftc.teamcode.util.VariableStateUtil
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class TagSS(
    hardwareMap : HardwareMap,
    webcamName: String = "Webcam 1"
) {
    private var aprilTag: AprilTagProcessor? = null
    private var visionPortal: VisionPortal? = null
    private val cameraPosition: Position = Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0)
    private val cameraOrientation = YawPitchRollAngles(AngleUnit.DEGREES, 0.0, -90.0, 0.0, 0)
    private val cameraRes = Size(640, 480)
    var lastTag = 0
    init {initAprilTag(hardwareMap, webcamName)}
    fun currentTag(): Int {
        val tags: ArrayList<AprilTagDetection?>? = aprilTag?.detections
        if (tags != null) {
            for (detections in tags) {
                if (detections?.metadata != null) {
                   lastTag = detections.id
                   return detections.id
                }
            }
        } else {
            VariableStateUtil.tagList.add(lastTag)
        }
        return 0
    }
    fun tagListDat(): MutableList<Int> = VariableStateUtil.tagList
    fun tagListSize(): Int = VariableStateUtil.tagList.size
    fun lastTag(): Int = lastTag
    private fun initAprilTag(hardwareMap: HardwareMap, webcamName: String) {
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