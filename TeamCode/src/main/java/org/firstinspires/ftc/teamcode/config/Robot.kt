package org.firstinspires.ftc.teamcode.config

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.math.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.seattlesolvers.solverslib.controller.PIDFController
import com.seattlesolvers.solverslib.hardware.motors.Motor
import com.seattlesolvers.solverslib.hardware.motors.MotorEx
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup
import com.seattlesolvers.solverslib.hardware.servos.ServoEx
import com.seattlesolvers.solverslib.hardware.servos.ServoExGroup
import com.seattlesolvers.solverslib.util.InterpLUT
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants

class Robot(
    private val hardwareMap: HardwareMap,
) {
    class LED(hardwareMap: HardwareMap) {
        private val led: ServoEx = ServoEx(hardwareMap, "LED")
        init {
            led.setPwm(PwmControl.PwmRange(500.0, 2500.0))
        }
        fun green() {
            led.set(0.4)
        }
        fun off() {
            led.set(0.0)
        }
        fun flash(count: Int): Command = Groups.repeat(Commands.instant { green(); off() }, count)
    }
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
    val led by lazy { LED(hardwareMap) }
    // Intake
    val intakeMCT: Double = 0.00001
    val intakeM1: MotorEx = MotorEx(hardwareMap,"intake1",Motor.GoBILDA.BARE).setCachingTolerance(intakeMCT); val intakeDef = 0.4
    val intakeM2: MotorEx = MotorEx(hardwareMap,"intake2",Motor.GoBILDA.BARE).setCachingTolerance(intakeMCT); val intakeFire = 0.6
    val intakeMG: MotorGroup = MotorGroup(intakeM1, intakeM2)
    val intakeMPIDF = PIDFController(0.0005, 0.0, 0.000, 0.00035)
    val pollenBS: ServoEx = ServoEx(hardwareMap, "pollen").setCachingTolerance(0.0001)
    // Shooter
    val fireM: MotorEx = MotorEx(hardwareMap,"fire",Motor.GoBILDA.BARE).setCachingTolerance(0.00001)
    val hoodDS: ServoEx = ServoEx(hardwareMap, "hood").setCachingTolerance(0.00000001)
    var fireMPIDF = PIDFController(0.00175, 0.0, 0.000, 0.00035)
    // Flower DeScorer
    val flowerDS1: ServoEx = ServoEx(hardwareMap,"fds1").setCachingTolerance(0.0) // Control Hub Servo 1
    val flowerDS2: ServoEx = ServoEx(hardwareMap,"fds2").setCachingTolerance(0.0) // Expansion Hub Servo 1
    val flowerDSG: ServoExGroup = ServoExGroup(flowerDS1, flowerDS2)

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
    //        add(Pose(81.0,9.0).distance(refPose), 0.45) // ~ Pos
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
    //        add(Pose(31.49,127.47).distance(refPose), -39.0)
    //        add(Pose(62.71,124.92).distance(refPose), -58.0)
    //        add(Pose(56.13, 127.68).distance(refPose), -60.0)
    //        add(Pose(86.5, 133.2).distance(refPose), -90.0)
    //        add(Pose(101.37, 131.5).distance(refPose), -111.0)
    //        add(Pose(120.48, 129.16).distance(refPose), -129.0)
    //    }.createLUT()

    //    downFireHeadBlueTab.apply {
    //        add(Pose(16.4,14.55).distance(refPose),25.0)
    //        add(Pose(46.35,13.06).distance(refPose), 45.0)
    //        add(Pose(65.25,10.73).distance(refPose), 61.0)
    //        add(Pose(81.0,9.0).distance(refPose), 90.0) // ~ Pos
    //        add(Pose(93.07,20.92).distance(refPose),112.0)
    //        add(Pose(112.4,11.79).distance(refPose),129.0)
    //    }.createLUT()

    //    upFireHeadRedTab.apply {
    //        add(Pose(28.72,121.8).distance(refPose), -50.0)
    //        add(Pose(59.0,133.4).distance(refPose), -90.0)
    //        add(Pose(70.35,127.75).distance(refPose), -107.0)
    //        add(Pose(82.45,126.9).distance(refPose), -122.0)
    //        add(Pose(87.98,128.6).distance(refPose), -128.0)
    //        add(Pose(109.64,127.11).distance(refPose), -143.0)
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
    //        add(Pose(81.0,9.0).distance(refPose), 0.1) // ~ Pos
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