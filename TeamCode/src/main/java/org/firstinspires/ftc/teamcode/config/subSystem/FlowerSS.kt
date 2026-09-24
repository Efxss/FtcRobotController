package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import org.firstinspires.ftc.teamcode.config.Robot

class FlowerSS(private val robot: Robot) {
    private val resetPos = 0.1
    private val kickPos = 0.325
    private fun exe(): Command = Groups.sequential(
        Commands.instant { robot.flowerS.set(kickPos) },
        Commands.waitMs(200.0).setPriority(10),
        Commands.instant { robot.flowerS.set(resetPos) },
        Commands.waitMs(350.0).setPriority(10)
    )
    fun deFlower(): Command = Groups.repeat(exe(),4)
    fun reset() = robot.flowerS.set(resetPos)
}