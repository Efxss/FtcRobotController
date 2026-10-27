package org.firstinspires.ftc.teamcode.config.subSystem

//import java.util.EnumMap
//import com.pedropathing.ivy.pedro.PedroCommands
//import com.pedropathing.math.Pose
//import kotlin.math.abs
//import com.seattlesolvers.solverslib.util.InterpLUT
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.ivy.groups.Groups.sequential
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.util.MathUtil
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

class FiringSS(
    private val robot: Robot,
    private val follower: Follower,
    private val alliance: Robot.Alliance,
    private val intakeSS: IntakeSS,
    private val servoBlock: Double = 0.18,
    private val servoGo: Double = 0.42
) {
    init {
        robot.fireM.inverted = true
    }
    private var fieldHalf: Robot.FieldHalf = Robot.FieldHalf.TOP
    private var power: Double = 0.0
    private var currentVel: Double = 0.0
    fun letGo(): Command {
        return sequential(
            Commands.instant { robot.pollenBS.set(servoGo) },
            //Commands.waitMs(1250.0),
            Commands.waitMs(1500.0),
            Commands.instant { robot.pollenBS.set(servoBlock) },
        ).requiring(robot.pollenBS)
    }
    fun execFiring(): Command {
        //val turnHalf = EnumMap<Robot.FieldHalf, Command>(Robot.FieldHalf::class.java)
        //fun topBlueCell(): Command {
        //    fun turn(): Command {
        //        return PedroCommands.hold(follower,Pose(
        //            follower.pose().x(),
        //            follower.pose().y(),
        //            Math.toRadians(robot.upFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose))))
        //        )
        //    }
        //    return Groups.parallel(intakeSpd(robot.upFireHeadBlueTab),turn())
        //}
        //fun bottomBlueCell(): Command {
        //    fun turn(): Command {
        //        return PedroCommands.hold(follower, Pose(
        //            follower.pose().x(),
        //            follower.pose().y(),
        //            Math.toRadians(robot.downFireHeadBlueTab.get(robot.follower.pose().distance(robot.refPose))))
        //        )
        //    }
        //    return Groups.parallel(intakeSpd(robot.downFireHeadBlueTab),turn())
        //}
        //fun topRedCell(): Command {
        //    fun turn(): Command {
        //        return PedroCommands.hold(follower,Pose(
        //            follower.pose().x(),
        //            follower.pose().y(),
        //            Math.toRadians(robot.upFireHeadRedTab.get(robot.follower.pose().distance(robot.refPose))))
        //        )
        //    }
        //    return Groups.parallel(intakeSpd(robot.upFireHeadRedTab),turn())
        //}
        //fun bottomRedCell(): Command {
        //    fun turn(): Command {
        //        return PedroCommands.hold(follower,Pose(
        //            follower.pose().x(),
        //            follower.pose().y(),
        //            Math.toRadians(robot.downFireHeadRedTab.get(robot.follower.pose().distance(robot.refPose))))
        //        )
        //    }
        //    return Groups.parallel(intakeSpd(robot.downFireHeadRedTab),turn())
        //}
        //when (alliance) {
        //    Robot.Alliance.BLUE -> {
        //        turnHalf[Robot.FieldHalf.TOP] = topBlueCell()
        //        turnHalf[Robot.FieldHalf.BOTTOM] = bottomBlueCell()
        //    }
        //    Robot.Alliance.RED -> {
        //        turnHalf[Robot.FieldHalf.TOP] = topRedCell()
        //        turnHalf[Robot.FieldHalf.BOTTOM] = bottomRedCell()
        //    }
        //}
        //fun turnHalfFun(): Command = Commands.match({fieldHalf}, turnHalf)
        return Groups.parallel(
            letGo(),
            //turnHalfFun(),
            //Groups.sequential(
            //    Commands.waitMs(5000.0),
            //    Commands.instant { intakeSS.velocity = robot.intakeDef }
            //)
        )
    }
    fun calcFiring(): Command {
        return Commands.infinite {
            this.currentVel = robot.fireM.correctedVelocity
            //this.power = robot.fireMPIDF.calculate(this.currentVel, 0.3).coerceIn(0.0,1.0) // Example to clamp val
            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(VariableStateUtil.wheelPower))
            robot.hoodDS.set(VariableStateUtil.hoodAngle) // MAX IS 0.65
            //when (alliance) {
            //    Robot.Alliance.BLUE -> {
            //        if (fieldHalf == Robot.FieldHalf.TOP) {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.upFireBlueTab.get(follower.pose().distance(robot.refPose))))
            //        }
            //        else {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.downFireBlueTab.get(follower.pose().distance(robot.refPose))))
            //        }
            //    }
            //    Robot.Alliance.RED -> {
            //        if (fieldHalf == Robot.FieldHalf.TOP) {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.upFireRedTab.get(follower.pose().distance(robot.refPose))))
            //        }
            //        else {
            //            this.power = robot.fireMPIDF.calculate(this.currentVel, MathUtil.pidfPower(robot.downFireRedTab.get(follower.pose().distance(robot.refPose))))
            //        }
            //    }
            //}
            robot.fireM.set(this.power)
        }
    }
    ////fun calcHalf(): Command = Commands.infinite { fieldHalf = if (follower.pose().y() >= 72) Robot.FieldHalf.TOP else Robot.FieldHalf.BOTTOM }
    //private fun intakeSpd(tab: InterpLUT): Command {
    //    val fastFireRange = 0.0..30.0
    //    val fastFire = 1.0
    //    val medFireRange = 30.0..60.0
    //    val medFire = 0.9
    //    val slowFireRange = 60.0..90.0
    //    val slowFire = 0.8
    //    var remain: Double
    //    return Commands.instant {
    //        remain = tab.get(robot.follower.pose().distance(robot.refPose)) - robot.follower.pose().heading()
    //        when (abs(remain)) {
    //            in fastFireRange -> intakeSS.velocity = fastFire
    //            in medFireRange -> intakeSS.velocity = medFire
    //            in slowFireRange -> intakeSS.velocity = slowFire
    //            else -> intakeSS.velocity = robot.intakeDef
    //        }
    //    }
    //}
    fun getHalf(): Robot.FieldHalf = this.fieldHalf
    fun getPower(): Double = this.power
    fun getVelocity(): Double = this.currentVel
    fun reset() {
        robot.pollenBS.set(this.servoBlock)
        robot.hoodDS.set(0.0)
    }
}