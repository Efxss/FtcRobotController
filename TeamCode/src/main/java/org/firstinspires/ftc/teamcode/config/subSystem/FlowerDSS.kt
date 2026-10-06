package org.firstinspires.ftc.teamcode.config.subSystem

import org.firstinspires.ftc.teamcode.config.Robot

class FlowerDSS(
    private val robot: Robot
) {
    private val servoBack = 0.0
    private val servoGo = 0.33
    //fun go(): Command = Commands.instant { robot.flowerDSG.set(servoGo) }
    //fun back(): Command = Commands.instant { robot.flowerDSG.set(servoBack) }
}