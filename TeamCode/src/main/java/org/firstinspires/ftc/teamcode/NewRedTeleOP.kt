package org.firstinspires.ftc.teamcode

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min


@TeleOp(name = "Red TeleOP (NEW)", group = "Main Red")
class NewRedTeleOP : OpMode() {
    var panels: TelemetryManager? = null
    private val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    private var handleTelemetry: Job? = null
    private var runDetections: Job?   = null
    private var outTakeCalc: Job?     = null
    private var manualFire: Job?      = null
    private var actVision: Job?       = null
    private var runIntake: Job?       = null
    private var patDect: Job?         = null
    private var actLift: Job?         = null
    private val startPose = Pose(72.0, 72.0, Math.toRadians(0.0))
    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer
    private lateinit var outTake1:  DcMotorEx
    private lateinit var outTake2:  DcMotorEx
    private lateinit var liftLeft:  DcMotorEx
    private lateinit var liftRight: DcMotorEx
    private lateinit var intakeServo1: CRServo
    private lateinit var intakeServo2: CRServo
    private lateinit var bowlServo: Servo
    private lateinit var camServo: Servo
    private lateinit var limelight: Limelight3A
    private lateinit var colorSensor: ColorSensor
    var ord = arrayOf("N", "N", "N")
    private var pathState: Int = 0
    private var dispensingState = 0
    private var isCentering = false
    private var rightBumperPressed = false
    var endgameTogglePressed = false
    var slowModeTogglePressed = false
    var isSlowMode = false
    private val pidP = 8.05
    private val pidI = 0.6
    private val pidD = 0.9
    private val pidF = 0.01
    private var velocityModeInitialized = false
    private var velocityPowerScale = 0.95
    private var intake = 0
    var isSeen = false
    object ServoPositions {
        // Loading positions
        const val LOAD_P1 = 0.0
        const val LOAD_P2 = 0.075
        const val LOAD_P3 = 0.148

        // Firing/dispensing positions
        const val FIRE_P1 = 0.114
        const val FIRE_P2 = 0.1845
        const val FIRE_P3 = 0.258

        // Camera servo positions
        const val CAM_OPEN = 0.5
        const val CAM_CLOSED = 0.255

        const val INTAKE_ON = 1.0
        const val INTAKE_REVERSE = -1.0
        const val INTAKE_OFF = 0.0
    }
    object AprilTagIds {
        const val GPP_ORDER = 21
        const val PGP_ORDER = 22
        const val PPG_ORDER = 23
        const val RED_DEPO =  24
    }
    object DepoCenter {
        const val DESIRED_TAG_WIDTH_PX = 110
        const val ROTATE_POWER = 0.2
        const val CAM_WIDTH_PX = 1280
        const val CAM_HEIGHT_PX = 720
        const val CENTER_DEADZONE = 15
        const val KP_ROTATE = 0.003
        var OUTTAKE_SPEED = 0.20
    }
    object EndGame {
        const val LIFTMAX = 11400
        const val SLOWMODE = 1000
        const val NORMALSPEED = 0.6
        const val SLOWSPEED = 0.2
        var ISENDGAME = 0
    }
    override fun init() {
        initializeHardware()
        initializePedroPathing()
    }

    override fun start() {
        opmodeTimer.resetTimer()
        follower.startTeleOpDrive()
        outTakeCalc = scope.launch {
            while (isActive) {
                outTakePower()
                delay(5)
            }
        }
        runIntake = scope.launch {
            while (isActive) {
                handleIntake()
                delay(5)
            }
        }
        runDetections = scope.launch {
            while (isActive) {
                handleDetections()
                delay(25)
            }
        }
    }
    
    override fun loop() {
        follower.update()
        //var rotate = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.5
        var rotate = if(gamepad1.left_bumper) 1.0 else if (gamepad1.right_bumper) -1.0 else 0.0
        var forward = -gamepad1.left_stick_y.toDouble()
        var strafe = gamepad1.right_stick_x.toDouble()

        follower.setMaxPower(0.6)
        follower.setTeleOpDrive(
            forward,
            strafe,
            rotate,
            true
        )
    }
    
    override fun stop() {
        outTakeCalc?.cancel()
        runIntake?.cancel()
        runDetections?.cancel()
    }

    private fun handleDetections() {
        var r = colorSensor.red();var g = colorSensor.green();var b = colorSensor.blue()
        panels?.debug("Red", r)
        panels?.debug("Green", g)
        panels?.debug("Blue", b)
        if (g <= 85 && b <= 110) {
            panels?.debug("None")
            isSeen = false
        }
        if (g >= 85 && b >= 110) {
            panels?.debug("Purple")
            if (!isSeen) {
                val slot = nextSlot()
                if (slot != -1) {
                    ord[slot] = "P"
                    advanceBowl(slot)
                }
                isSeen = true
            }
        }
        if (g >= 100) {
            panels?.debug("Green")
            if (!isSeen) {
                val slot = nextSlot()
                if (slot != -1) {
                    ord[slot] = "G"
                    advanceBowl(slot)
                }
                isSeen = true
            }
        }
    }
    private fun outTakePower() {
        val result: LLResult? = limelight.latestResult
        if (result == null || !result.isValid) {
            return
        }
        val fiducialResults = result.fiducialResults
        val target = fiducialResults.firstOrNull { it.fiducialId == AprilTagIds.RED_DEPO }
        if (target == null) {
            return
        }
        val corners = target.targetCorners
        if (corners != null && corners.size >= 4) {
            val tagHeightPx = hypot(
                corners[3][0] - corners[0][0],
                corners[3][1] - corners[0][1]
            )
            var heightCalc = tagHeightPx - 85
            var scale = heightCalc * 0.000685
            var powerResult = 0.27 - scale
            DepoCenter.OUTTAKE_SPEED = powerResult
        }
        outTake1.power = DepoCenter.OUTTAKE_SPEED
        outTake2.power = DepoCenter.OUTTAKE_SPEED
    }
    private fun handleIntake() {
        if (gamepad1.cross) {
            intakeServo1.power = ServoPositions.INTAKE_ON
            intakeServo2.power = ServoPositions.INTAKE_ON
        } else if (gamepad1.circle) {
            intakeServo1.power = ServoPositions.INTAKE_REVERSE
            intakeServo2.power = ServoPositions.INTAKE_REVERSE
        }
        else {
            intakeServo1.power = ServoPositions.INTAKE_OFF
            intakeServo2.power = ServoPositions.INTAKE_OFF
        }
    }
    private fun initializeHardware() {
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        liftLeft = hardwareMap.get(DcMotorEx::class.java, "liftLeft")
        liftRight = hardwareMap.get(DcMotorEx::class.java, "liftRight")
        intakeServo1 = hardwareMap.get(CRServo::class.java, "intakeServo1")
        intakeServo2 = hardwareMap.get(CRServo::class.java, "intakeServo2")
        bowlServo = hardwareMap.get(Servo::class.java, "bowlServo")
        camServo = hardwareMap.get(Servo::class.java, "camServo")
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        colorSensor = hardwareMap.get(ColorSensor::class.java, "ColorSensor")
        camServo.position = ServoPositions.CAM_CLOSED
        bowlServo.position = ServoPositions.LOAD_P1
        setupMotorDirections()
        setupPIDFCoefficients()
        resetEncoders()
        setupVision()
    }
    private fun setupVision() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
        limelight.start()
    }
    private fun setupMotorDirections() {
        listOf(intakeServo2, outTake2, liftLeft)
            .forEach { it.direction = DcMotorSimple.Direction.REVERSE }
    }
    private fun setupPIDFCoefficients() {
        listOf(outTake1, outTake2)
            .forEach { it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF) }
    }
    private fun resetEncoders() {
        listOf(outTake1, outTake2).forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }
        listOf(liftLeft, liftRight).forEach { motor ->
            motor.targetPosition = EndGame.LIFTMAX
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }
    private fun initializePedroPathing() {
        panels = PanelsTelemetry.telemetry

        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()

        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()
    }
    private fun ensureVelocityMode() {
        if (!velocityModeInitialized) {
            outTake1.mode = DcMotor.RunMode.RUN_USING_ENCODER
            outTake2.mode = DcMotor.RunMode.RUN_USING_ENCODER
            velocityModeInitialized = true
        }
    }
    private fun powerToTicksPerSecond(motor: DcMotorEx, power: Double): Double {
        val clipped = max(-1.0, min(1.0, power))
        val maxRpm = motor.motorType.maxRPM
        val tpr = motor.motorType.ticksPerRev
        val maxTicksPerSec = (maxRpm * tpr) / 60.0
        return clipped * velocityPowerScale * maxTicksPerSec
    }
    private fun setMotorVelocityFromPseudoPower(motor: DcMotorEx, power: Double) {
        ensureVelocityMode()
        motor.velocity = powerToTicksPerSecond(motor, power)
    }
    private fun clip(v: Double, min: Double, max: Double): Double {
        return max(min, min(max, v))
    }
    private fun nextSlot(): Int {
        return when {
            ord[0] == "N" -> 0
            ord[1] == "N" -> 1
            ord[2] == "N" -> 2
            else -> -1 // Array is full
        }
    }
    private fun advanceBowl(slot: Int) {
        bowlServo.position = when (slot) {
            0 -> ServoPositions.LOAD_P2
            1 -> ServoPositions.LOAD_P3
            else -> bowlServo.position // Stay in place if full
        }
    }
}
