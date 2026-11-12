package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import org.firstinspires.ftc.teamcode.config.Robot

class FlowerDSS(
    private val robot: Robot
) {
    private val servoBack = 0.0
    private val servoGo = 0.49
    init {
        robot.flowerDS1.inverted = true
        robot.flowerDS2.inverted = false
        robot.flowerDSG.set(this.servoBack)
    }
    fun down(): Command = Commands.instant {
        robot.flowerDSG.set(this.servoGo)
    }
    fun back(): Command = Commands.instant {
        robot.flowerDSG.set(this.servoBack)
    }
}