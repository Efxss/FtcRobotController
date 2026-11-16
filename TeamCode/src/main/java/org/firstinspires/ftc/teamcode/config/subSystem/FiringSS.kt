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
import com.seattlesolvers.solverslib.util.InterpLUT
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
                    //Math.toRadians(robot.downFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose))))
                )
            }
            return sequential(turning(),stop(),turn(),notTurning())
        }
        return Groups.sequential(
            //robot.led.flash(3),
            Commands.instant { StateUtil.isFiring = true },
            Commands.instant { intakeSS.speed = robot.intakeDef },
            turnFun(),
            Commands.waitMs(500.0),
            letGo(),
            //Groups.sequential(
            //    Commands.waitMs(850.0),
            //    Commands.instant { intakeSS.speed = robot.intakeDef }
            //)
        )
    }

    fun calcFiring(): Command {
        return Commands.infinite {
            this.currentVel = robot.fireM.correctedVelocity
            //this.power = robot.fireMPIDF.calculate(this.currentVel, 0.3).coerceIn(0.0,1.0) // Example to clamp val
            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(StateUtil.wheelPower))
            robot.hoodDS.set(StateUtil.hoodAngle) // MAX IS 0.65
            //when (alliance) {
            //    Robot.Alliance.BLUE -> {
            //        if (fieldHalf == Robot.FieldHalf.TOP) {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.upFireBlueTab.get(robot.follower.pose().distance(robot.refPose))))
            //        }
            //        else {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.downFireBlueTab.get(robot.follower.pose().distance(robot.refPose))))
            //        }
            //    }
            //    Robot.Alliance.RED -> {
            //        if (fieldHalf == Robot.FieldHalf.TOP) {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.upFireRedTab.get(robot.follower.pose().distance(robot.refPose))))
            //        }
            //        else {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.downFireRedTab.get(robot.follower.pose().distance(robot.refPose))))
            //        }
            //    }
            //}
            robot.fireM.set(this.power)
        }
    }

    fun calcHalf(): Command = Commands.infinite { fieldHalf = if (robot.follower.pose().y() >= 72) Robot.FieldHalf.TOP else Robot.FieldHalf.BOTTOM }

    private fun intakeSpd(tab: InterpLUT): Command {
        val fastFireRange = 0.0..30.0
        val fastFire = 1.0
        val medFireRange = 30.0..60.0
        val medFire = 0.9
        val slowFireRange = 60.0..90.0
        val slowFire = 0.8
        var remain: Double
        return Commands.instant {
            remain = tab.get(robot.follower.pose().distance(robot.refPose)) - robot.follower.pose().heading()
            when (abs(remain)) {
                in fastFireRange -> intakeSS.speed = fastFire
                in medFireRange -> intakeSS.speed = medFire
                in slowFireRange -> intakeSS.speed = slowFire
                else -> intakeSS.speed = robot.intakeDef
            }
        }
    }

    fun getHalf(): Robot.FieldHalf = this.fieldHalf

    fun getPower(): Double = this.power

    fun getVelocity(): Double = this.currentVel

    fun reset() {
        robot.pollenBS.set(this.servoBlock)
        robot.hoodDS.set(0.0)
    }
}