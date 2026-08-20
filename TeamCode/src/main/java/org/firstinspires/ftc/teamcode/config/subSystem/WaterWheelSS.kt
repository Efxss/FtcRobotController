package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

class WaterWheelSS(
    hardwareMap: HardwareMap
) {
    private val wheelServo: CRServo = hardwareMap.get(CRServo::class.java, "wheel")
    val runWheel: Command = Command.build()
        .setStart { wheelServo.power = -0.50 }
        .setEnd { wheelServo.power = 0.0 }
}