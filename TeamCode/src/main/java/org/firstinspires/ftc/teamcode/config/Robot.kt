package org.firstinspires.ftc.teamcode.config

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.pedropathing.math.Pose
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
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
    val bottomBlueRefPose: Pose = Pose(84.0,54.0)
    val topBlueRefPose: Pose = Pose(84.0,85.0)
    fun initPedro() { follower = Constants.create(hardwareMap) }
    // Hardware
    val led by lazy { LED(hardwareMap) }
    val pnpt by lazy { hardwareMap["pnpt"] as GoBildaPinpointDriver }
    // Intake
    var intakeMCT: Double = 0.0
    val intakeM1 by lazy { MotorEx(hardwareMap,"intake1",Motor.GoBILDA.BARE).setCachingTolerance(intakeMCT) as MotorEx }; val intakeDef = 0.4
    val intakeM2 by lazy { MotorEx(hardwareMap,"intake2",Motor.GoBILDA.BARE).setCachingTolerance(intakeMCT) as MotorEx }; val intakeFire = 0.35
    val intakeMG by lazy { MotorGroup(intakeM1, intakeM2) }
    val intakeMPIDF = PIDFController(0.0005, 0.0, 0.000, 0.00035)
    // Shooter
    val fireM by lazy { MotorEx(hardwareMap,"fire",Motor.GoBILDA.BARE).setCachingTolerance(0.0) as MotorEx }
    val hoodDS by lazy { ServoEx(hardwareMap, "hood").setCachingTolerance(0.0) as ServoEx }
    val pollenBS by lazy { ServoEx(hardwareMap, "pollen").setCachingTolerance(0.0) as ServoEx }
    var fireMPIDF = PIDFController(0.0075, 0.0, 0.000, 0.00035)
    // Flower DeScorer
    var flowerDSC = 0.0
    val flowerDS1 by lazy { ServoEx(hardwareMap,"fds1").setCachingTolerance(flowerDSC) as ServoEx }  // Control Hub Servo 1
    val flowerDS2 by lazy { ServoEx(hardwareMap,"fds2").setCachingTolerance(flowerDSC) as ServoEx }  // Expansion Hub Servo 1
    val flowerDSG by lazy { ServoExGroup(flowerDS1, flowerDS2) }

    // Tables and refs

    // Flywheel Power
    val fireTab = InterpLUT()

    // Hood Angle
    val fireHoodTab = InterpLUT()

    // Objects, Classes and Functions
    fun genTab() {
        // Flywheel Power
        fireTab.apply {
            add(47.0, 0.69)
            add(79.0, 0.72)
        }.createLUT()

        // Hood Angle
        fireHoodTab.apply {
            add(47.0, 0.125)
            add(79.0, 0.12)
        }.createLUT()
    }
}