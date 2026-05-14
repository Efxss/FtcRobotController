package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

class IntakeSS(
    hardwareMap: HardwareMap
) {
    private val intakeMotor : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "intake")
    val runIntake : Command = instant { intakeMotor.power = 0.8 }
    val stopIntake : Command = instant { intakeMotor.power = 0.0 }
    val reverseIntake : Command = instant { intakeMotor.power = -0.8 }
}