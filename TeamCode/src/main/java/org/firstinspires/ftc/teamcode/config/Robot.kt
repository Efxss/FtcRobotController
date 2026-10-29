package org.firstinspires.ftc.teamcode.config

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import com.seattlesolvers.solverslib.util.InterpLUT
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants

class Robot(
    private val hardwareMap: HardwareMap,
) {
    enum class Alliance {BLUE, RED}
    enum class FieldHalf {TOP,BOTTOM}
    // PedroPathing and Panels
    var panels: TelemetryManager? = null
    fun initPanels() { panels = PanelsTelemetry.telemetry }
    lateinit var follower: Follower
    val p: PoseFactory = PoseFactory.degrees()
    val refPose: Pose = Pose(0.0,0.0)
    fun initPedro() { follower = Constants.create(hardwareMap) }
    // Hardware

    // Intake
    val intakeMC: Double = 0.00001
    val intakeM1: MotorEx = MotorEx(hardwareMap,"intake1",Motor.GoBILDA.BARE).setCachingTolerance(intakeMC); val intakeDef = 0.4
    val intakeM2: MotorEx = MotorEx(hardwareMap,"intake2",Motor.GoBILDA.BARE).setCachingTolerance(intakeMC); val intakeFire = 0.6
    val intakeMG: MotorGroup = MotorGroup(intakeM1, intakeM2)
    val intakeMPIDF = PIDFController(0.0009, 0.0, 0.0, 0.00018)
    val pollenBS: ServoEx = ServoEx(hardwareMap, "pollen").setCachingTolerance(0.0001)
    // Shooter
    val fireM: MotorEx = MotorEx(hardwareMap,"fire",Motor.GoBILDA.BARE).setCachingTolerance(0.00001)
    val hoodDS: ServoEx = ServoEx(hardwareMap, "hood").setCachingTolerance(0.00000001)
    var fireMPIDF = PIDFController(0.0013, 0.0, 0.0001, 0.00018)
    // Flower DeScorer
    private val flowerDSC = 0.1
    //private val flowerDS1: ServoEx = ServoEx(hardwareMap,"FDS1").setCachingTolerance(flowerDSC)
    //private val flowerDS2: ServoEx = ServoEx(hardwareMap,"FDS2").setCachingTolerance(flowerDSC)
    //val flowerDSG: ServoExGroup = ServoExGroup(flowerDS1,flowerDS2)

    // Tables and refs

    // Flywheel Power
    val upFireBlueTab = InterpLUT()
    val downFireBlueTab = InterpLUT()
    val upFireRedTab = InterpLUT()
    val downFireRedTab = InterpLUT()

    // Cell Heading
    val upFireHeadBlueTab = InterpLUT()
    val downFireHeadBlueTab = InterpLUT()
    val upFireHeadRedTab = InterpLUT()
    val downFireHeadRedTab = InterpLUT()

    // Hood Angle
    val upFireHoodBlueTab = InterpLUT()
    val downFireHoodBlueTab = InterpLUT()
    val upFireHoodRedTab = InterpLUT()
    val downFireHoodRedTab = InterpLUT()
    // Objects, Classes and Functions
    //fun genTab() {
    //    // Flywheel Power
    //    upFireBlueTab.apply {
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //    }.createLUT()

    //    downFireBlueTab.apply {
    //        add(Pose(81.0,9.0).distance(refPose), 0.56) // ~ Pos
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //    }.createLUT()

    //    upFireRedTab.apply {
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //    }.createLUT()

    //    downFireRedTab.apply {
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //    }.createLUT()

    //    // Cell Heading
    //    upFireHeadBlueTab.apply {
    //        add(Pose(85.8, 133.2).distance(refPose), 270.0)
    //    }.createLUT()

    //    downFireHeadBlueTab.apply {
    //        add(Pose(81.0,9.0).distance(refPose), 90.0) // ~ Pos
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //    }.createLUT()

    //    upFireHeadRedTab.apply {
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //    }.createLUT()

    //    downFireHeadRedTab.apply {
    //        add(Pose(16.4,12.0).distance(refPose), 44.0)
    //        add(Pose(33.7,12.0).distance(refPose), 56.0)
    //        add(Pose(62.2,8.0).distance(refPose), 90.0)
    //        add(Pose(80.0,12.0).distance(refPose), 124.0)
    //        add(Pose(96.5,17.5).distance(refPose), 137.0)
    //        add(Pose(110.4,12.0).distance(refPose), 141.0)
    //    }.createLUT()

    //    // Hood Angle
    //    upFireHoodBlueTab.apply {
    //        add(Pose(85.8, 133.2).distance(refPose), 270.0)
    //    }.createLUT()

    //    downFireHoodBlueTab.apply {
    //        add(Pose(81.0,9.0).distance(refPose), 0.17) // ~ Pos
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //    }.createLUT()

    //    upFireHoodRedTab.apply {
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //        add(Pose(0.0,0.0).distance(refPose), 0.1)
    //    }.createLUT()

    //    downFireHoodRedTab.apply {
    //        add(Pose(16.4,12.0).distance(refPose), 44.0)
    //    }.createLUT()
    //}
}