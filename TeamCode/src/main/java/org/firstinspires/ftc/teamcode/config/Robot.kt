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
    val bottomRedRefPose: Pose = Pose(57.0,55.0)
    val topRedRefPose: Pose = Pose(57.0,85.0)
    val bottomBlueRefPose: Pose = Pose(84.0,55.0)
    val topBlueRefPose: Pose = Pose(84.0,85.0)
    fun initPedro() { follower = Constants.create(hardwareMap) }
    // Hardware
    val led by lazy { LED(hardwareMap) }
    // Intake
    val intakeMCT: Double = 0.00001
    val intakeM1 by lazy { MotorEx(hardwareMap,"intake1",Motor.GoBILDA.BARE).setCachingTolerance(intakeMCT) as MotorEx }; val intakeDef = 0.4
    val intakeM2 by lazy { MotorEx(hardwareMap,"intake2",Motor.GoBILDA.BARE).setCachingTolerance(intakeMCT) as MotorEx }; val intakeFire = 0.6
    val intakeMG by lazy { MotorGroup(intakeM1, intakeM2) }
    val intakeMPIDF = PIDFController(0.0005, 0.0, 0.000, 0.00035)
    val pollenBS by lazy { ServoEx(hardwareMap, "pollen").setCachingTolerance(0.0001) as ServoEx }
    // Shooter
    val fireM by lazy { MotorEx(hardwareMap,"fire",Motor.GoBILDA.BARE).setCachingTolerance(0.00001) as MotorEx }
    val hoodDS by lazy { ServoEx(hardwareMap, "hood").setCachingTolerance(0.00000001) as ServoEx }
    var fireMPIDF = PIDFController(0.00175, 0.0, 0.000, 0.00035)
    // Flower DeScorer
    val flowerDS1 by lazy { ServoEx(hardwareMap,"fds1").setCachingTolerance(0.0) as ServoEx }  // Control Hub Servo 1
    val flowerDS2 by lazy { ServoEx(hardwareMap,"fds2").setCachingTolerance(0.0) as ServoEx }  // Expansion Hub Servo 1
    val flowerDSG by lazy { ServoExGroup(flowerDS1, flowerDS2) }

    // Tables and refs

    // Flywheel Power
    val upFireBlueTab = InterpLUT()
    val downFireBlueTab = InterpLUT()
    val upFireRedTab = InterpLUT()
    val downFireRedTab = InterpLUT()

    // Hood Angle
    val upFireHoodBlueTab = InterpLUT()
    val downFireHoodBlueTab = InterpLUT()
    val upFireHoodRedTab = InterpLUT()
    val downFireHoodRedTab = InterpLUT()
    // Objects, Classes and Functions
    fun genTab() {
        // Flywheel Power
        //upFireBlueTab.apply {
        //    add(Pose(0.0,0.0).distance(topBlueRefPose), 0.1)
        //    add(Pose(0.0,0.0).distance(topBlueRefPose), 0.1)
        //}.createLUT()

        //downFireBlueTab.apply {
        //    add(Pose(81.0,9.0).distance(bottomBlueRefPose), 0.45) // ~ Pos
        //    add(Pose(0.0,0.0).distance(bottomBlueRefPose), 0.1)
        //}.createLUT()

        //upFireRedTab.apply {
        //    add(Pose(0.0,0.0).distance(topRedRefPose), 0.1)
        //    add(Pose(0.0,0.0).distance(topRedRefPose), 0.1)
        //}.createLUT()

        //downFireRedTab.apply {
        //    add(Pose(0.0,0.0).distance(bottomRedRefPose), 0.1)
        //    add(Pose(0.0,0.0).distance(bottomRedRefPose), 0.1)
        //}.createLUT()

        // Hood Angle
        //upFireHoodBlueTab.apply {
        //    add(Pose(85.8, 133.2).distance(topBlueRefPose), 270.0)
        //}.createLUT()

        //downFireHoodBlueTab.apply {
        //    add(Pose(81.0,9.0).distance(bottomBlueRefPose), 0.1) // ~ Pos
        //    add(Pose(0.0,0.0).distance(bottomBlueRefPose), 0.1)
        //}.createLUT()

        //upFireHoodRedTab.apply {
        //    add(Pose(0.0,0.0).distance(topRedRefPose), 0.1)
        //    add(Pose(0.0,0.0).distance(topRedRefPose), 0.1)
        //}.createLUT()

        //downFireHoodRedTab.apply {
        //    add(Pose(16.4,12.0).distance(bottomRedRefPose), 44.0)
        //}.createLUT()
    }
}