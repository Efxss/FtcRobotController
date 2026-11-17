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
    private var currentVel: Double = 0.0
    private var pidfVelocity: Double = 0.0
    private val servoBlock: Double = 0.18
    private val servoGo: Double = 0.42
    var power: Double = 0.0
    var hoodAngle: Double = 0.0

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
            Commands.instant { intakeSS.speed = robot.intakeFire },
            turnFun(),
            letGo(),
            Commands.waitMs(150.0),
            Commands.instant { intakeSS.speed = robot.intakeDef },
        )
    }

    fun calcFiring(autoCalc: Boolean): Command {
        return Commands.infinite {
            this.currentVel = robot.fireM.correctedVelocity
            if (autoCalc) {
                when (alliance) {
                    Robot.Alliance.BLUE -> {
                        when (this.fieldHalf) {
                            Robot.FieldHalf.TOP -> {
                                this.pidfVelocity = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.fireTab.get(robot.follower.pose().distance(robot.topBlueRefPose))))
                                this.hoodAngle = robot.fireHoodTab.get(robot.follower.pose().distance(robot.topBlueRefPose))
                            }
                            Robot.FieldHalf.BOTTOM -> {
                                this.pidfVelocity = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.fireTab.get(robot.follower.pose().distance(robot.bottomBlueRefPose))))
                                this.hoodAngle = robot.fireHoodTab.get(robot.follower.pose().distance(robot.bottomBlueRefPose))
                            }
                        }
                    }
                    Robot.Alliance.RED -> {
                        when (this.fieldHalf) {
                            Robot.FieldHalf.TOP -> {
                                this.pidfVelocity = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.fireTab.get(robot.follower.pose().distance(robot.topRedRefPose))))
                                this.hoodAngle = robot.fireHoodTab.get(robot.follower.pose().distance(robot.topRedRefPose))
                            }
                            Robot.FieldHalf.BOTTOM -> {
                                this.pidfVelocity = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.fireTab.get(robot.follower.pose().distance(robot.bottomRedRefPose))))
                                this.hoodAngle = robot.fireHoodTab.get(robot.follower.pose().distance(robot.bottomRedRefPose))
                            }
                        }
                    }
                }
            } else {
                this.pidfVelocity = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(this.power))
            }
            robot.hoodDS.set(this.hoodAngle)
            robot.fireM.set(this.pidfVelocity)
        }
    }

    fun calcHalf(): Command = Commands.infinite { fieldHalf = if (robot.follower.pose().y() >= 72) Robot.FieldHalf.TOP else Robot.FieldHalf.BOTTOM }

    fun getHalf(): Robot.FieldHalf = this.fieldHalf

    fun getVelocity(): Double = this.currentVel

    fun reset() {
        robot.pollenBS.set(this.servoBlock)
        robot.hoodDS.set(0.0)
    }
}