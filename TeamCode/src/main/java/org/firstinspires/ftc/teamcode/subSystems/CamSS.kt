package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.MathUtil

/** A SubSystem made for controlling the Cam on the robot */
class CamSS (
    hardwareMap : HardwareMap,
    val firingPos : Double,
    val homePos : Double
) {
    private val cam : Servo = hardwareMap.get(Servo::class.java, "Cam")

    /** Calling this function will return the Cam to its declared home position */
    fun home() {
        cam.position = homePos
    }

    /** Calling this function will return the Cam to its declared firing position */
    fun fire() {
        cam.position = firingPos
    }

    /** This function will return the position as a String if the position is within error of a known position */
    fun position() : String {
        val camPos = cam.position

        return when {
            MathUtil.closeTo(camPos, firingPos) -> "Firing"
            MathUtil.closeTo(camPos, homePos) -> "Home"
            else -> "Unknown"
        }
    }

    /** This function will return the raw position of the Cam as a double */
    fun rawPosition() : Double = cam.position
}