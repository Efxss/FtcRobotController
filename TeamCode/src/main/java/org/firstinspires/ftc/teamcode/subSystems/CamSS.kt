package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import java.lang.Thread.sleep

class CamSS(
    hardwareMap: HardwareMap
) {
    private val cam: Servo = hardwareMap.get(Servo::class.java, "Cam")

    fun home() {
        cam.position = 0.0
    }

    fun fire() {
        cam.position = 1.0
    }

    fun fireCam() {
        fire()
        sleep(1000)
        home()
    }

    fun position() : Double {
        return cam.position
    }
}