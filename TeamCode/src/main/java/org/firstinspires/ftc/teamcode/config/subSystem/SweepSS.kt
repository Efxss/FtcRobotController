package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.sequential
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo

class SweepSS(
    hardwareMap: HardwareMap
) {
    private val sweepServo: Servo = hardwareMap.get(Servo::class.java, "sweep")
    init { sweepServo.position = 0.23 }
    private fun returnSweepServo(): Command { return instant { sweepServo.position = 0.23 } }
    private fun runSweepServoOut(): Command { return instant { sweepServo.position = 0.53 } }
    fun execSweepServo(): Command {
        return sequential(
            runSweepServoOut(),
            waitMs(3000.0),
            returnSweepServo()
        )
    }
    fun position(): Double = sweepServo.position
}