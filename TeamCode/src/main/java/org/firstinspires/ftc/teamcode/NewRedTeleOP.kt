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
import kotlin.math.abs
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
    private var firing: Job?          = null
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
    private var isDispensing = false
    private var rightBumperPressed = false
    var endgameTogglePressed = false
    var slowModeTogglePressed = false
    var isSlowMode = false
    var depoCentered = false
    var lastTriggerPressed = false
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
        const val CAM_HEIGHT_PX = 960
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
    object Timing {
        const val DISPENSE_INITIAL_DELAY = 100L
        const val BOWL_MOVE_DELAY = 1000L
        const val CAM_OPEN_DELAY = 400L
        const val CAM_CLOSE_DELAY = 1500L
        const val DETECTION_COOLDOWN = 1500L
        const val OUTTAKE_DELAY = 800L
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
        firing = scope.launch {
            while (isActive) {
                handleFiring()
                delay(25)
            }
        }
    }
    
    override fun loop() {
        follower.update()
        var rotate = if(gamepad1.left_bumper) 0.5 else if (gamepad1.right_bumper) -0.5 else 0.0
        var forward = -gamepad1.left_stick_y.toDouble()
        var strafe = gamepad1.right_stick_x.toDouble()

        follower.setMaxPower(0.8)
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

    private suspend fun handleFiring() {
        val triggerPressed = gamepad1.right_trigger > 0.5

        if (triggerPressed && !lastTriggerPressed && !isDispensing) {
            depoCentered = !depoCentered
        }

        lastTriggerPressed = triggerPressed

        if (depoCentered && !isDispensing) {
            centerDepo()
        }
    }
    private suspend fun centerDepo() {
        if (isDispensing) return  // Guard clause

        follower.setMaxPower(0.1)
        val result: LLResult? = limelight.latestResult
        val fiducialResults = result?.fiducialResults
        val target = fiducialResults?.firstOrNull { it.fiducialId == AprilTagIds.RED_DEPO }

        if (target == null) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
            return
        }

        val xErrPx: Double = target.targetXPixels - (DepoCenter.CAM_WIDTH_PX / 2.0)

        if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)

            isDispensing = true  // Lock before dispensing
            try {
                dispensingState = 1
                executeDispensing()
            } finally {
                isDispensing = false  // Always unlock, even on error
                depoCentered = false  // Reset state (don't toggle, just set to false)
                isCentering = false
            }
            return
        }

        val rotationPower = clip(xErrPx * DepoCenter.KP_ROTATE, -0.3, 0.3)
        follower.setTeleOpDrive(0.0, 0.0, -rotationPower, false)
    }
    private suspend fun executeDispensing() {
        delay(100) // OUTTAKE_DELAY

        val isFull = ord.none { it == "N" }

        if (isFull) {
            val dispenseSequence = listOf(
                ServoPositions.FIRE_P3,
                ServoPositions.FIRE_P2,
                ServoPositions.FIRE_P1
            )
            executeDispenseSequence(dispenseSequence)
            isCentering = false
        }

        // Lower outtake and reset
        delay(100)
        bowlServo.position = ServoPositions.LOAD_P1
        delay(100)

        // Reset state
        dispensingState = 0
        ord = arrayOf("N", "N", "N")
    }
    private suspend fun executeDispenseSequence(positions: List<Double>) {
        delay(Timing.DISPENSE_INITIAL_DELAY)

        positions.forEach { position ->
            bowlServo.position = position
            delay(Timing.BOWL_MOVE_DELAY)

            camServo.position = ServoPositions.CAM_OPEN
            delay(Timing.CAM_OPEN_DELAY)

            camServo.position = ServoPositions.CAM_CLOSED
            delay(Timing.CAM_CLOSE_DELAY)
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
        val targetY = target.targetYPixels
        var powerResult = 0.000204*targetY+0.13
        DepoCenter.OUTTAKE_SPEED = powerResult
        outTake1.power = DepoCenter.OUTTAKE_SPEED
        outTake2.power = DepoCenter.OUTTAKE_SPEED
        panels?.debug("targetY", targetY)
        panels?.debug("OutTake 1 Power", outTake1.power)
        panels?.debug("OutTake 2 Power", outTake2.power)
        panels?.debug("OUTTAKE_SPEED", DepoCenter.OUTTAKE_SPEED)
        panels?.update(telemetry)
    }
    private fun handleIntake() {
        val isFull = ord.none { it == "N" }
        /*if (gamepad1.cross) {
            intakeServo1.power = ServoPositions.INTAKE_ON
            intakeServo2.power = ServoPositions.INTAKE_ON
        } else if (gamepad1.circle) {
            intakeServo1.power = ServoPositions.INTAKE_REVERSE
            intakeServo2.power = ServoPositions.INTAKE_REVERSE
        }
        else {
            intakeServo1.power = ServoPositions.INTAKE_OFF
            intakeServo2.power = ServoPositions.INTAKE_OFF
        }*/
        if (isFull) {
            intakeServo1.power = ServoPositions.INTAKE_REVERSE
            intakeServo2.power = ServoPositions.INTAKE_REVERSE
        } else {
            intakeServo1.power = ServoPositions.INTAKE_ON
            intakeServo2.power = ServoPositions.INTAKE_ON
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
        bowlServo.position = ServoPositions.LOAD_P2
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
    private fun setMotorVelocityFromPseudoPower(motor: DcMotorEx, power: Double) {
        ensureVelocityMode()
        motor.velocity = powerToTicksPerSecond(motor, power)
    }
    private fun powerToTicksPerSecond(motor: DcMotorEx, power: Double): Double {
        val clipped = max(-1.0, min(1.0, power))
        val maxRpm = motor.motorType.maxRPM
        val tpr = motor.motorType.ticksPerRev
        val maxTicksPerSec = (maxRpm * tpr) / 60.0
        return clipped * velocityPowerScale * maxTicksPerSec
    }
    private fun clip(v: Double, min: Double, max: Double): Double {
        return max(min, min(max, v))
    }
    private suspend fun handleDetections() {
        var r = colorSensor.red();var g = colorSensor.green();var b = colorSensor.blue()
        if (g <= 85 && b <= 110) {
            isSeen = false
        }
        if (g >= 85 && b >= 110) {
            if (!isSeen) {
                val slot = nextSlot()
                if (slot != -1) {
                    ord[slot] = "P"
                    advanceBowl(slot)
                    delay(Timing.DETECTION_COOLDOWN)
                }
                isSeen = true
            }
        }
        if (g >= 100) {
            if (!isSeen) {
                val slot = nextSlot()
                if (slot != -1) {
                    ord[slot] = "G"
                    advanceBowl(slot)
                    delay(Timing.DETECTION_COOLDOWN)
                }
                isSeen = true
            }
        }
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
            2 -> ServoPositions.FIRE_P3
            else -> bowlServo.position // Stay in place if full
        }
    }
}
