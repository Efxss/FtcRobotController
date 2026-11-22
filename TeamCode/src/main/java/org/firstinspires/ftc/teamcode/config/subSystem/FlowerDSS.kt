package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import org.firstinspires.ftc.teamcode.config.Robot

class FlowerDSS(
    private val robot: Robot
) {
    private val servoBack = 0.0
    private val servoGo = 0.475

    init {
        robot.flowerDS.inverted = true
        robot.flowerDS.set(servoBack)
    }

    fun down(): Command = Commands.instant {
        robot.flowerDS.set(this.servoGo)
    }

    fun back(): Command = Commands.instant {
        robot.flowerDS.set(this.servoBack)
    }
}