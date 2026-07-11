package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

class PushServoSS(
    hardwareMap: HardwareMap
) {
    private val pushServo: CRServo = hardwareMap.get(CRServo::class.java, "push")
    val runPush: Command = Command.build()
        .setStart { pushServo.power = 1.0 }
        .setEnd { pushServo.power = 0.0 }
    val stopPush: Command = Command.build()
        .setStart { pushServo.power = 0.0 }
        .setEnd { pushServo.power = 1.0 }
}