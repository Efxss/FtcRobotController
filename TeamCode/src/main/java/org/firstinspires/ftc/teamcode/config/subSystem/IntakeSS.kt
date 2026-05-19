package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

class IntakeSS(
    hardwareMap: HardwareMap
) {
    private val intakeMotor : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "intake")
    private val intakeSpeed = 0.8
    private var lastPower : Double = Double.NaN
    private fun setPower(power : Double) { if (power != lastPower) { intakeMotor.power = power
            lastPower = power } }
    val runIntakeCommand : Command = instant { setPower(intakeSpeed) }
    val stopIntakeCommand : Command = instant { setPower(0.0) }
    val reverseIntakeCommand : Command = instant { setPower(-intakeSpeed) }
    fun runIntakeFun() { setPower(intakeSpeed) }
    fun stopIntakeFun() { setPower(0.0) }
    fun reverseIntakeFun() { setPower(-intakeSpeed) }

}