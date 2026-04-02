package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.util.MathUtil
import java.lang.Thread.sleep

class CamSS (
    hardwareMap : HardwareMap
) {
    private val cam : Servo = hardwareMap.get(Servo::class.java, "Cam")
    val firingPos : Double = 0.5
    val homePos : Double = 0.0

    fun home() {
        cam.position = homePos
    }

    fun fire() {
        cam.position = firingPos
    }

    fun rawPosition() : Double = cam.position

    fun position() : String {
        val camPos = cam.position

        return when {
            MathUtil.closeTo(camPos, firingPos) -> "Firing"
            MathUtil.closeTo(camPos, homePos) -> "Home"
            else -> "Unknown"
        }
    }
}