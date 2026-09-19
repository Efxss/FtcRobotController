package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import org.firstinspires.ftc.teamcode.config.Robot

class FlowerSS(private val robot: Robot) {
    private fun exe(): Command = Groups.sequential(
        Commands.instant { robot.flowerS.set(0.4) },
        Commands.waitMs(250.0).setPriority(10),
        Commands.instant { robot.flowerS.set(0.0) },
        Commands.waitMs(250.0).setPriority(10)
    )
    fun deFlower(): Command = Groups.repeat(exe(),3)
    fun reset() = robot.flowerS.set(0.0)
}