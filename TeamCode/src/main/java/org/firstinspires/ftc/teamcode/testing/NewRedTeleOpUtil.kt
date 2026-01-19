package org.firstinspires.ftc.teamcode.testing

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Servo
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.util.LimelightUtil
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt

@Disabled
class NewRedTeleOpUtil : OpMode() {

    var panels: TelemetryManager? = null
    private val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())

    private var runDetections: Job? = null
    private var outTakeCalc: Job? = null
    private var runIntake: Job? = null
    private var firing: Job? = null

    private val startPose = Pose(72.0, 72.0, Math.toRadians(0.0))
    private lateinit var follower: Follower
    private lateinit var pathTimer: Timer
    private lateinit var actionTimer: Timer
    private lateinit var opmodeTimer: Timer

    private lateinit var outTake1: DcMotorEx
    private lateinit var outTake2: DcMotorEx
    private lateinit var liftLeft: DcMotorEx
    private lateinit var liftRight: DcMotorEx
    private lateinit var intakeServo1: CRServo

    private lateinit var bowlServo: Servo
    private lateinit var camServo: Servo
    private lateinit var limelight: Limelight3A

    private lateinit var colorSensor: ColorSensor
    private lateinit var statusLEDR: DigitalChannel
    private lateinit var statusLEDG: DigitalChannel

    @Volatile
    var ord = arrayOf("N", "N", "N")

    // State flags
    @Volatile var isDispensing = false
    private var rightBumperPressed = false
    private var dPadDownPressed = false
    private var isSlowMode = false
    private var isSeen = false

    // PIDF config you already had
    private val pidP = 150.0
    private val pidI = 0.0
    private val pidD = 0.0
    private val pidF = 13.5

    private var velocityModeInitialized = false
    private var velocityPowerScale = 1.0

    object ServoPositions {
        // Loading positions
        const val LOAD_P1 = 0.004
        const val LOAD_P2 = 0.080
        const val LOAD_P3 = 0.153

        // Firing/dispensing positions
        const val FIRE_P1 = 0.118
        const val FIRE_P2 = 0.1885
        const val FIRE_P3 = 0.042

        // Camera servo positions
        const val CAM_OPEN = 0.44
        const val CAM_CLOSED = 0.255

        const val INTAKE_ON = 1.0
        const val INTAKE_REVERSE = -1.0
        const val INTAKE_OFF = 0.0
    }

    object DepoCenter {
        var OUTTAKE_SPEED = 0.20
    }

    object EndGame {
        const val LIFTMAX = 11400
        const val SLOWMODE = 1000
        const val NORMALSPEED = 1.0
        const val SLOWSPEED = 0.2
    }

    object Timing {
        const val DETECTION_COOLDOWN = 1500L
        var nextDetectAllowedMs = 0L
    }

    // ---- Variable drive speed tuning ----
    private object DriveScale {
        const val DEADBAND = 0.06   // ignore tiny stick noise
        const val MIN_SCALE = 0.20  // minimum drive authority when barely moving
        const val EXPO = 1.6        // 1.0 = linear, >1 = softer near center
    }

    /**
     * Returns 0..1 based on how far the driver is commanding motion.
     * Uses both translation (forward + strafe) and rotation (rotate) so turning-only still scales.
     */
    private fun computeDriveScale(forward: Double, strafe: Double, rotate: Double): Double {
        val transMag = sqrt(forward * forward + strafe * strafe).coerceIn(0.0, 1.0)
        val rotMag = abs(rotate).coerceIn(0.0, 1.0)
        var mag = max(transMag, rotMag)

        // Deadband
        if (mag < DriveScale.DEADBAND) mag = 0.0

        // Normalize after deadband to 0..1
        val norm = if (mag == 0.0) 0.0
        else ((mag - DriveScale.DEADBAND) / (1.0 - DriveScale.DEADBAND)).coerceIn(0.0, 1.0)

        // Expo curve
        val curved = norm.pow(DriveScale.EXPO)

        // Map to MIN_SCALE..1.0
        return (DriveScale.MIN_SCALE + (1.0 - DriveScale.MIN_SCALE) * curved).coerceIn(0.0, 1.0)
    }

    override fun init() {
        initializeHardware()
        initializePedroPathing()

        // Hook helper to the existing hardware you already init'd
        LimelightUtil.init(limelight, bowlServo, camServo)
        LimelightUtil.setPipeline(0)
    }

    override fun start() {
        resetServos()
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

        // ---- DRIVE CONTROLS ----
        // Slow mode on LEFT bumper (hold)
        isSlowMode = gamepad1.left_bumper
        val baseMax = if (isSlowMode) 0.30 else 0.80

        // If the auto center+fire routine is running, don't fight it with manual drive commands
        if (!isDispensing) {
            val forward = -gamepad1.left_stick_y.toDouble()
            val strafe = gamepad1.right_stick_x.toDouble()

            // Triggers = analog turning
            val turnInput = (gamepad1.right_trigger - gamepad1.left_trigger).toDouble()
            val turnDeadband = 0.05
            val maxTurn = 0.8
            val rotate = if (abs(turnInput) < turnDeadband) 0.0 else turnInput * maxTurn

            // Variable speed based on stick/trigger demand
            val demandScale = computeDriveScale(forward, strafe, rotate)
            follower.setMaxPower(baseMax * demandScale)

            follower.setTeleOpDrive(forward, strafe, rotate, true)

            // Telemetry (optional but handy)
            panels?.debug("Drive baseMax", baseMax)
            panels?.debug("Drive demandScale", demandScale)
            panels?.debug("Drive maxPower", baseMax * demandScale)
        } else {
            follower.setMaxPower(0.0)
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
        }

        // ---- TELEMETRY ----
        panels?.debug("OutTake 1 Power", outTake1.power)
        panels?.debug("OutTake 2 Power", outTake2.power)
        panels?.debug("OutTake 1 velocity", outTake1.velocity)
        panels?.debug("OutTake 2 velocity", outTake2.velocity)
        panels?.debug("OUTTAKE_SPEED", DepoCenter.OUTTAKE_SPEED)
        panels?.debug("ord[0]", ord[0])
        panels?.debug("ord[1]", ord[1])
        panels?.debug("ord[2]", ord[2])
        panels?.debug("Run Time", runtime)
        panels?.update(telemetry)

        // ---- Manual bowl nudge (kept from your original) ----
        if (gamepad1.dpad_down && !dPadDownPressed) {
            bowlServo.position -= 0.1
        } else if (!gamepad1.dpad_down && dPadDownPressed) {
            bowlServo.position += 0.1
        }
        dPadDownPressed = gamepad1.dpad_down

        // ---- Endgame kill / lift mode (kept from your original logic) ----
        if (gamepad1.dpad_up && gamepad1.triangle) {
            if (liftLeft.currentPosition < EndGame.SLOWMODE || liftRight.currentPosition < EndGame.SLOWMODE) {
                listOf(liftLeft, liftRight).forEach { it.power = EndGame.NORMALSPEED }
            } else if (liftLeft.currentPosition > EndGame.SLOWMODE || liftRight.currentPosition > EndGame.SLOWMODE) {
                listOf(liftLeft, liftRight).forEach { it.power = EndGame.SLOWSPEED }
            } else if (liftLeft.currentPosition > EndGame.LIFTMAX || liftRight.currentPosition > EndGame.LIFTMAX) {
                listOf(liftLeft, liftRight).forEach { it.power = 0.0 }
            }

            outTakeCalc?.cancel()
            outTake1.power = 0.0
            outTake2.power = 0.0

            runIntake?.cancel()
            intakeServo1.power = 0.0
        }
    }

    override fun stop() {
        outTakeCalc?.cancel()
        runIntake?.cancel()
        runDetections?.cancel()
        firing?.cancel()
        LimelightUtil.stop()
    }

    // -------------------------
    // Center + Fire on RIGHT bumper using LimelightUtil
    // -------------------------
    private suspend fun handleFiring() {
        val centerPressed = gamepad1.right_bumper

        if (centerPressed && !rightBumperPressed && !isDispensing) {
            isDispensing = true
            try {
                follower.setMaxPower(0.15)

                // Center on RED_DEPO and fire loaded balls based on ord[]
                LimelightUtil.centerAndFire(
                    tagId = LimelightUtil.Tags.RED_DEPO,
                    follower = follower,
                    slots = ord
                )

                // reset after firing
                ord = arrayOf("N", "N", "N")
                bowlServo.position = ServoPositions.LOAD_P1
            } finally {
                isDispensing = false
            }
        }

        rightBumperPressed = centerPressed
    }

    // -------------------------
    // Outtake power calc
    // -------------------------
    private fun outTakePower() {
        val p = LimelightUtil.calculateOuttakePower(LimelightUtil.Tags.RED_DEPO) ?: return
        DepoCenter.OUTTAKE_SPEED = p
        outTake1.power = p
        outTake2.power = p
    }

    private fun handleIntake() {
        val isFull = if (gamepad1.circle) true else ord.none { it == "N" }

        if (isFull) {
            intakeServo1.power = ServoPositions.INTAKE_REVERSE
            bothLEDon()
        } else {
            intakeServo1.power = ServoPositions.INTAKE_ON
            bothLEDoff()
        }
    }

    private suspend fun handleDetections() {
        if (isDispensing) return

        val g = colorSensor.green()
        val b = colorSensor.blue()

        // "Clear" => re-arm immediately (even during cooldown)
        if (g <= 80 && b <= 110) {
            isSeen = false
            return
        }

        // cooldown
        val now = System.currentTimeMillis()
        if (now < Timing.nextDetectAllowedMs) return

        if (!isSeen) {
            val slot = nextSlot()
            if (slot != -1) {
                // Your original logic: g>=110 => "G" else "P"
                ord[slot] = if (g >= 110) "G" else "P"

                delay(200)
                advanceBowl(slot)

                Timing.nextDetectAllowedMs = now + Timing.DETECTION_COOLDOWN
            }
            isSeen = true
        }
    }

    private fun nextSlot(): Int {
        return when {
            ord[0] == "N" -> 0
            ord[1] == "N" -> 1
            ord[2] == "N" -> 2
            else -> -1
        }
    }

    private fun advanceBowl(slot: Int) {
        bowlServo.position = when (slot) {
            0 -> ServoPositions.LOAD_P2
            1 -> ServoPositions.LOAD_P3
            2 -> ServoPositions.FIRE_P2
            else -> bowlServo.position
        }
    }

    // -------------------------
    // Hardware + setup
    // -------------------------
    private fun initializeHardware() {
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        liftLeft = hardwareMap.get(DcMotorEx::class.java, "liftLeft")
        liftRight = hardwareMap.get(DcMotorEx::class.java, "liftRight")
        intakeServo1 = hardwareMap.get(CRServo::class.java, "intakeServo1")
        bowlServo = hardwareMap.get(Servo::class.java, "bowlServo")
        camServo = hardwareMap.get(Servo::class.java, "camServo")
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        colorSensor = hardwareMap.get(ColorSensor::class.java, "ColorSensor")
        statusLEDR = hardwareMap.get(DigitalChannel::class.java, "statusLEDR")
        statusLEDG = hardwareMap.get(DigitalChannel::class.java, "statusLEDG")

        statusLEDR.mode = DigitalChannel.Mode.OUTPUT
        statusLEDG.mode = DigitalChannel.Mode.OUTPUT

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

    private fun bothLEDoff() {
        listOf(statusLEDR, statusLEDG).forEach { it.state = true }
    }

    private fun bothLEDon() {
        listOf(statusLEDR, statusLEDG).forEach { it.state = false }
    }

    private fun setupMotorDirections() {
        listOf(outTake2, liftLeft).forEach { it.direction = DcMotorSimple.Direction.REVERSE }
    }

    private fun setupPIDFCoefficients() {
        listOf(outTake1, outTake2).forEach { it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF) }
    }

    private fun resetEncoders() {
        listOf(outTake1, outTake2).forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        listOf(liftLeft, liftRight).forEach { motor ->
            motor.targetPosition = EndGame.LIFTMAX
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    private fun resetServos() {
        camServo.position = ServoPositions.CAM_CLOSED
        bowlServo.position = ServoPositions.FIRE_P2
    }

    private fun initializePedroPathing() {
        panels = PanelsTelemetry.telemetry

        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer().apply { resetTimer() }

        // keep your default if you want, otherwise set to empty:
        // ord = arrayOf("N", "N", "N")
        ord = arrayOf("P", "G", "P")

        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()
    }

    // Kept from your original helper utilities (in case you still need them somewhere)
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
}
