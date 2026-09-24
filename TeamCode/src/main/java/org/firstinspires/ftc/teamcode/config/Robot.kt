package org.firstinspires.ftc.teamcode.config

import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import com.seattlesolvers.solverslib.util.InterpLUT
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.PoseUtil
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

class Robot(
    hardwareMap: HardwareMap,
) {
    enum class Alliance {BLUE, RED}
    enum class OpMode {AUTO, TELEOP}
    // PedroPathing
    lateinit var follower: Follower
    val p: PoseFactory = PoseFactory.degrees()
    fun initPedro(hardwareMap: HardwareMap,opmode: OpMode) {
        when (opmode) {
            OpMode.AUTO -> {
                follower = Constants.create(hardwareMap)
                follower.setPose(PoseUtil.startPose)
            }
            OpMode.TELEOP -> {
                val startPose = VariableStateUtil.endOfAutoPose
                follower = Constants.create(hardwareMap)
                follower.setPose(startPose)
            }
        }
    }
    // Hardware

    val intakeM: MotorEx = MotorEx(hardwareMap,"intake",Motor.GoBILDA.BARE).setCachingTolerance(0.2)
    //val fireM: MotorEx = MotorEx(hardwareMap,"fire",Motor.GoBILDA.BARE).setCachingTolerance(0.05)
    val flowerS: ServoEx = ServoEx(hardwareMap, "flower").setCachingTolerance(0.1);val flowerLoop = 4

    // Tables and refs
    val refPose: Pose = Pose(0.0,0.0)
    val upFireBlueTab = InterpLUT()
    val downFireBlueTab = InterpLUT()
    val upFireRedTab = InterpLUT()
    val downFireRedTab = InterpLUT()
    val upFireHeadBlueTab = InterpLUT()
    val downFireHeadBlueTab = InterpLUT()
    val upFireHeadRedTab = InterpLUT()
    val downFireHeadRedTab = InterpLUT()
    // Objects, Classes and Functions
    fun genTab() {
        // Fire Power
        upFireBlueTab.apply {
            add(Pose(0.0,0.0).distance(refPose), 0.1)
            add(Pose(0.0,0.0).distance(refPose), 0.1)
        }.createLUT()

        downFireBlueTab.apply {
            add(Pose(0.0,0.0).distance(refPose), 0.1)
            add(Pose(0.0,0.0).distance(refPose), 0.1)
        }.createLUT()

        upFireRedTab.apply {
            add(Pose(0.0,0.0).distance(refPose), 0.1)
            add(Pose(0.0,0.0).distance(refPose), 0.1)
        }.createLUT()

        downFireRedTab.apply {
            add(Pose(0.0,0.0).distance(refPose), 0.1)
            add(Pose(0.0,0.0).distance(refPose), 0.1)
        }.createLUT()

        // Turn Heading
        upFireHeadBlueTab.apply {
            add(Pose(85.8, 133.2).distance(refPose), 270.0)
        }.createLUT()

        downFireHeadBlueTab.apply {
            add(Pose(0.0,0.0).distance(refPose), 0.1)
            add(Pose(0.0,0.0).distance(refPose), 0.1)
        }.createLUT()

        upFireHeadRedTab.apply {
            add(Pose(0.0,0.0).distance(refPose), 0.1)
            add(Pose(0.0,0.0).distance(refPose), 0.1)
        }.createLUT()

        downFireHeadRedTab.apply {
            add(Pose(16.4,12.0).distance(refPose), 44.0)
            add(Pose(33.7,12.0).distance(refPose), 56.0)
            add(Pose(62.2,8.0).distance(refPose), 90.0)
            add(Pose(80.0,12.0).distance(refPose), 124.0)
            add(Pose(96.5,17.5).distance(refPose), 137.0)
            add(Pose(110.4,12.0).distance(refPose), 141.0)
        }.createLUT()
    }
}