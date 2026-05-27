package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap

class IntakeSS(
    hardwareMap : HardwareMap
) {
    private val intakeMotor : DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "intake")
    init {
        intakeMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    private val intakeVelocity = 1120.0
    private var lastVelocity : Double = Double.NaN
    private fun setVelocity(velocity : Double) { if (velocity != lastVelocity) { intakeMotor.velocity = velocity
            lastVelocity = velocity } }
    val runIntakeCommand : Command = Command.build()
        .setStart { setVelocity(intakeVelocity) }
        .setEnd { setVelocity(0.0) }
    val reverseIntakeCommand : Command = Command.build()
        .setStart { setVelocity(-intakeVelocity) }
        .setEnd { setVelocity(0.0) }
}