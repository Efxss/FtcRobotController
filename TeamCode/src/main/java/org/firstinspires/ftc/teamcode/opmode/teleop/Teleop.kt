package org.firstinspires.ftc.teamcode.opmode.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.configurables.annotations.Sorter
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.TeleOpMode

@Configurable
@TeleOp
class Teleop : TeleOpMode() {
    companion object {
        @JvmField
        @Sorter(sort = 0)
        var fkp: Double = 0.04
        @JvmField
        @Sorter(sort = 1)
        var fki: Double = 0.0
        @JvmField
        @Sorter(sort = 2)
        var fkd: Double = 0.0004
        @JvmField
        @Sorter(sort = 3)
        var fkf: Double = 0.0004
        @JvmField
        @Sorter(sort = 4)
        var privateHoodAngle: Double = 0.1
        @JvmField
        @Sorter(sort = 5)
        var privateWheelPower: Double = 0.61
        @JvmField
        @Sorter(sort = 6)
        var privateIntakeFire: Double = 0.3
    }
    override val alliance = Robot.Alliance.BLUE

    override fun onInit() {
    }

    override fun onStart() {
        firingSS.calcFiring(false).schedule()
    }

    override fun onLoop() {
        //if (!StateUtil.isTurning) {
        //    robot.follower.manual(dp)
        //    ManualDrive.driveOrHold(robot.follower, dp) // Do this to make it so when you stop holding the stick it holds position
        //}
        firingSS.hoodAngle = privateHoodAngle
        firingSS.power = privateWheelPower
        robot.intakeFire = privateIntakeFire
        robot.fireMPIDF.setPIDF(fkp, fki, fkd, fkf)
        robot.follower.update()
        elapsedTime.reset()
    }
}