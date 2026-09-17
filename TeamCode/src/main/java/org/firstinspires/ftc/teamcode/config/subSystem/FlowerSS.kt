package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.groups.Groups
import org.firstinspires.ftc.teamcode.config.Robot

class FlowerSS(robot: Robot) {
    fun deFlower(robot: Robot): Command {
        val run: Command = Command.build()
            .setStart { robot.flowerS.set(1.0) }
            .setEnd { robot.flowerS.set(0.0) }
            .setDone { robot.flowerS.rawPosition >= 1.0 }
        return Groups.repeat(run, robot.flowerLoop).requiring(robot.flowerS)
    }
}