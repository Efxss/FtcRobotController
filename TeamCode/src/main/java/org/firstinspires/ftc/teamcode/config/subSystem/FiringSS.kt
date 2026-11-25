package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands
import com.pedropathing.math.Pose
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.util.MathUtils
import com.seattlesolvers.solverslib.util.Timing
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
    private val initRampVel = 0.20
    private val lastRampVel = 0.22
    private val lastBallTimer: Timing.Timer = Timing.Timer(140, java.util.concurrent.TimeUnit.MILLISECONDS)
    private lateinit var refPose: Pose
    private var distance: Double = 0.0
    var power: Double = 0.78
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

    fun execFiring(autoTurn: Boolean): Command {
        fun turnFun(): Command {
            fun turning(): Command { return Commands.instant { StateUtil.isTurning = true } }
            fun stop(): Command = Commands.instant { robot.follower.stop() }
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
            fun turn(): Command {
                return PedroCommands.hold(robot.follower, Pose(
                    robot.follower.pose().x(),
                    robot.follower.pose().y(),
                    MathUtil.cellRadian(robot.follower.pose(), this.fieldHalf, this.alliance, this.robot))
                )
            }
            return sequential(turning(),stop(),turn(),notTurning())
        }
        return if (autoTurn) {
            sequential(
                //robot.led.flash(3),
                Commands.instant { StateUtil.isFiring = true },
                Commands.instant { intakeSS.speed = robot.intakeFire },
                turnFun(),
                letGo(),
                Commands.waitMs(150.0),
                Commands.instant { intakeSS.speed = robot.intakeDef },
            )
        } else {
            sequential(
                //robot.led.flash(3),
                Commands.instant { StateUtil.isFiring = true },
                Commands.instant { intakeSS.speed = robot.intakeFire },
                letGo(),
                Commands.waitMs(150.0),
                Commands.instant { intakeSS.speed = robot.intakeDef },
            )
        }
    }

    fun calcFiring(autoCalc: Boolean): Command {
        return Commands.infinite {
            this.currentVel = robot.fireM.correctedVelocity
            if (autoCalc) {
                refPose = when (alliance) {
                    Robot.Alliance.BLUE -> when (this.fieldHalf) {
                        Robot.FieldHalf.TOP -> robot.topBlueRefPose
                        Robot.FieldHalf.BOTTOM -> robot.bottomBlueRefPose
                    }
                    Robot.Alliance.RED -> when (this.fieldHalf) {
                        Robot.FieldHalf.TOP -> robot.topRedRefPose
                        Robot.FieldHalf.BOTTOM -> robot.bottomRedRefPose
                    }
                }
                distance = robot.follower.pose().distance(refPose)
                this.power = robot.fireTab.get(distance)
                this.hoodAngle = robot.fireHoodTab.get(distance)
                this.pidfVelocity = if (StateUtil.isFiring) {
                    lastBallTimer.start()
                    if (lastBallTimer.done()) robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(this.power + lastRampVel))
                    else robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(this.power))
                } else {
                    robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(this.power - initRampVel))
                }
            } else {
                this.pidfVelocity = if (StateUtil.isFiring) {
                    lastBallTimer.start()
                    if (lastBallTimer.done()) robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(this.power + lastRampVel))
                    else robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(this.power))
                } else {
                    robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(this.power - initRampVel))
                }
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