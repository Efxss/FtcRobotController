package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class SweepSS(
    hardwareMap: HardwareMap
) {
    private val sweepServo: Servo = hardwareMap.get(Servo::class.java, "sweep")
    val runSweep: Command = Command.build()
        .setStart { sweepServo.position = 1.0 }
        .setEnd { sweepServo.position = 0.0 }
}