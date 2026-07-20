package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.HardwareMap

class PushServoSS(
    hardwareMap: HardwareMap
) {
    private val pushServo: CRServo = hardwareMap.get(CRServo::class.java, "push")
    val runPush: Command = com.pedropathing.ivy.commands.Commands.instant { pushServo.power = 1.0 }
    val stopPush: Command = com.pedropathing.ivy.commands.Commands.instant { pushServo.power = 0.0 }
}