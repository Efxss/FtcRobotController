package org.firstinspires.ftc.teamcode.subSystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import java.lang.Thread.sleep

class CamSS(
    hardwareMap: HardwareMap
) {
    private val cam: Servo = hardwareMap.get(Servo::class.java, "cam")

    fun home() {
        cam.position = 0.255
    }

    fun fire() {
        cam.position = 0.44
    }

    fun fireCam() {
        fire()
        sleep(5)
        home()
    }
}