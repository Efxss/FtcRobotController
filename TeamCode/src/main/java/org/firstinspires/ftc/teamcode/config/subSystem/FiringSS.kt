package org.firstinspires.ftc.teamcode.config.subSystem

//import java.util.EnumMap
//import com.pedropathing.ivy.pedro.PedroCommands
//import com.pedropathing.math.Pose
//import kotlin.math.abs
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.math.Pose
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.MathUtils
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.util.MathUtil
import org.firstinspires.ftc.teamcode.config.util.StateUtil
import kotlin.math.abs

class FiringSS(
    private val robot: Robot,
    private val alliance: Robot.Alliance,
    private val intakeSS: IntakeSS,
) {
    init {
        robot.fireM.inverted = true
        robot.fireM.setRunMode(Motor.RunMode.RawPower)
    }
    private var fieldHalf: Robot.FieldHalf = Robot.FieldHalf.TOP
    private var power: Double = 0.0
    private var currentVel: Double = 0.0
    private val servoBlock: Double = 0.18
    private val servoGo: Double = 0.42

    fun letGo(): Command {
        return sequential(
            Commands.instant { robot.pollenBS.set(servoGo) },
            //Commands.waitMs(1250.0),
            Commands.waitMs(1000.0),
            Commands.instant { robot.pollenBS.set(servoBlock) },
            Commands.instant { StateUtil.isFiring = false },
        ).requiring(robot.pollenBS)
    }

    fun execFiring(): Command {
        fun turnFun(): Command {
            fun turning(): Command { return Commands.instant { StateUtil.isTurning = true } }
            fun notTurning(): Command {
                return Command.build()
                    .setDone {
                        val error = MathUtils.normalizeRadians(
                            robot.follower.pose().heading() -
                                    MathUtil.cellRadian(robot.follower.pose(), this.fieldHalf, this.alliance, this.robot),
                            false
                        )
                        abs(error) < Math.toRadians(3.0)
                    }
                    .setEnd { StateUtil.isTurning = false }
            }
            fun stop(): Command = Commands.instant { robot.follower.stop() }
            fun turn(): Command {
                return PedroCommands.hold(robot.follower, Pose(
                    robot.follower.pose().x(),
                    robot.follower.pose().y(),
                    MathUtil.cellRadian(robot.follower.pose(), this.fieldHalf, this.alliance, this.robot))
                )
            }
            return sequential(turning(),stop(),turn(),notTurning())
        }
        return Groups.sequential(
            //robot.led.flash(3),
            Commands.instant { StateUtil.isFiring = true },
            Commands.instant { intakeSS.speed = robot.intakeDef },
            turnFun(),
            letGo(),
        )
    }

    fun calcFiring(): Command {
        return Commands.infinite {
            this.currentVel = robot.fireM.correctedVelocity
            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(StateUtil.wheelPower))
            robot.hoodDS.set(StateUtil.hoodAngle) // MAX IS 0.65 maybe
            //when (alliance) {
            //    Robot.Alliance.BLUE -> {
            //        if (fieldHalf == Robot.FieldHalf.TOP) {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.upFireBlueTab.get(robot.follower.pose().distance(robot.topBlueRefPose))))
            //        }
            //        else {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.downFireBlueTab.get(robot.follower.pose().distance(robot.bottomBlueRefPose))))
            //        }
            //    }
            //    Robot.Alliance.RED -> {
            //        if (fieldHalf == Robot.FieldHalf.TOP) {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.upFireRedTab.get(robot.follower.pose().distance(robot.topRedRefPose))))
            //        }
            //        else {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.downFireRedTab.get(robot.follower.pose().distance(robot.bottomRedRefPose))))
            //        }
            //    }
            //}
            robot.fireM.set(this.power)
        }
    }

    fun calcHalf(): Command = Commands.infinite { fieldHalf = if (robot.follower.pose().y() >= 72) Robot.FieldHalf.TOP else Robot.FieldHalf.BOTTOM }

    fun getHalf(): Robot.FieldHalf = this.fieldHalf

    fun getPower(): Double = this.power

    fun getVelocity(): Double = this.currentVel

    fun reset() {
        robot.pollenBS.set(this.servoBlock)
        robot.hoodDS.set(0.0)
    }
}