package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import org.firstinspires.ftc.teamcode.config.Robot

class FlowerSS(robot: Robot) {
    private fun exe(robot: Robot): Command = Groups.sequential(
        Commands.instant { robot.flowerS.set(0.4) },
        Commands.waitMs(250.0).setPriority(10),
        Commands.instant { robot.flowerS.set(0.0) },
        Commands.waitMs(250.0).setPriority(10)
    )
    fun deFlower(robot: Robot): Command = Groups.repeat(exe(robot),3)
    fun reset(robot: Robot) = robot.flowerS.set(0.0)
}