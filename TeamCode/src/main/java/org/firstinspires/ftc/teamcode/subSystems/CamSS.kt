package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
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

    fun fireCam() {
        fire()
        sleep(1000)
        home()
    }

    fun rawPosition() : Double {
        return cam.position
    }
    fun position() : String {
        val camPos = cam.position

        return when {
            closeTo(camPos, firingPos) -> "Firing"
            closeTo(camPos, homePos) -> "Home"
            else -> "Unknown"
        }
    }

    fun closeTo(a: Double, b: Double, epsilon: Double = 0.001): Boolean {
        return kotlin.math.abs(a - b) < epsilon
    }
}