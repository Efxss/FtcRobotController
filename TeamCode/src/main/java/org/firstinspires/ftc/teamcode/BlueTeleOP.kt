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
import org.firstinspires.ftc.teamcode.util.Drawing
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "Blue TeleOP", group = "Main Blue")
class BlueTeleOP : OpMode() {
    var panels: TelemetryManager? = null
    val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    var handleTelemetry: Job? = null
    var runDetections: Job?   = null
    var outTakeCalc: Job?     = null
    var manualFire: Job?      = null
    var actVision: Job?       = null
    var runIntake: Job?       = null
    var patDect: Job?         = null
    var actLift: Job?         = null
    var firing: Job?          = null
    val startPose = Pose(72.0, 8.375, Math.toRadians(0.0))
    lateinit var follower: Follower
    lateinit var pathTimer: Timer
    lateinit var actionTimer: Timer
    lateinit var opmodeTimer: Timer
    lateinit var outTake1:  DcMotorEx
    lateinit var outTake2:  DcMotorEx
    lateinit var liftLeft:  DcMotorEx
    lateinit var liftRight: DcMotorEx
    lateinit var intakeServo1: CRServo
    lateinit var bowlServo: Servo
    lateinit var camServo: Servo
    lateinit var limelight: Limelight3A
    lateinit var colorSensor: ColorSensor
    lateinit var statusLEDR: DigitalChannel
    lateinit var statusLEDG: DigitalChannel
    @Volatile
    var ord = arrayOf("N", "N", "N")
    var pathState: Int = 0
    var dispensingState = 0
    var isCentering = false
    @Volatile
    var isDispensing = false
    var rightBumperPressed = false
    var endgameTogglePressed = false
    var slowModeTogglePressed = false
    var dPadRightPressed = false
    var dPadLeftPressed = false
    var squarePressed = false
    var isSlowMode = false
    var depoCentered = false
    var lastTriggerPressed = false
    val pidP = 150.0
    val pidI = 0.0
    val pidD = 0.0
    val pidF = 13.5
    var velocityModeInitialized = false
    var velocityPowerScale = 1.0
    var intake = 0
    var isSeen = false
    object ServoPositions {
        // Loading positions
        const val LOAD_P1 = 0.021
        const val LOAD_P2 = 0.087
        const val LOAD_P3 = 0.158

        // Firing/dispensing positions
        const val FIRE_P1 = 0.128
        const val FIRE_P2 = 0.195
        const val FIRE_P3 = 0.058

        // Camera servo positions
        const val CAM_OPEN = 0.44
        const val CAM_CLOSED = 0.255

        const val INTAKE_ON = 1.0
        const val INTAKE_REVERSE = -1.0
        const val INTAKE_OFF = 0.0
    }
    object AprilTagIds {
        const val BLUE_DEPO = 20
        const val GPP_ORDER = 21
        const val PGP_ORDER = 22
        const val PPG_ORDER = 23
    }
    object DepoCenter {
        const val DESIRED_TAG_WIDTH_PX = 80
        const val ROTATE_POWER = 0.2
        const val CAM_WIDTH_PX = 1280
        const val CAM_HEIGHT_PX = 960
        const val CENTER_DEADZONE = 13
        const val KP_ROTATE = 0.003
        var OUTTAKE_SPEED = 0.20
    }
    object EndGame {
        const val LIFTMAX = 1140
        const val SLOWMODE = 1000
        const val NORMALSPEED = 1.0
        const val SLOWSPEED = 0.2
        var ISENDGAME = 0
    }
    object Timing {
        const val DISPENSE_INITIAL_DELAY = 100L
        const val BOWL_MOVE_DELAY = 250L
        const val CAM_OPEN_DELAY = 140L
        const val CAM_CLOSE_DELAY = 170L
        const val DETECTION_COOLDOWN = 400L
        const val OUTTAKE_DELAY = 800L
        var nextDetectAllowedMs = 0L
    }
    override fun init() {
        initializeHardware()
        initializePedroPathing()
        Drawing.init()
    }

    override fun init_loop() {
        follower.update()
        Drawing.drawOnlyCurrent(follower)
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
                delay(3)
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

        Drawing.drawDebug(follower)

        follower.setMaxPower(if (isSlowMode) 0.3 else 0.8)
        follower.setTeleOpDrive(
            forward,
            strafe,
            rotate,
            true
        )

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

        if (gamepad1.square && !squarePressed) {
            ord = arrayOf("P", "P", "P")
        }
        squarePressed = gamepad1.square

        if (gamepad1.dpad_left && !dPadLeftPressed) {
            bowlServo.position -= 0.1
        } else if (!gamepad1.dpad_down && dPadLeftPressed) {
            bowlServo.position += 0.1
        }
        dPadLeftPressed = gamepad1.dpad_left
        if (gamepad1.dpad_right && !dPadRightPressed) {
            bowlServo.position += 0.1
        } else if (!gamepad1.dpad_right && dPadRightPressed) {
            bowlServo.position -= 0.1
        }
        dPadRightPressed = gamepad1.dpad_right

        if (gamepad1.dpad_up && gamepad1.triangle) {
            if (liftLeft.currentPosition < EndGame.SLOWMODE || liftRight.currentPosition < EndGame.SLOWMODE) {
                listOf(liftLeft, liftRight).forEach { motor ->
                    motor.power = EndGame.NORMALSPEED
                }
            } else if (liftLeft.currentPosition > EndGame.SLOWMODE || liftRight.currentPosition > EndGame.SLOWMODE) {
                listOf(liftLeft, liftRight).forEach { motor ->
                    motor.power = EndGame.SLOWSPEED
                }
            } else if (liftLeft.currentPosition > EndGame.LIFTMAX || liftRight.currentPosition > EndGame.LIFTMAX) {
                listOf(liftLeft, liftRight).forEach { motor ->
                    motor.power = 0.0
                }
            }
            outTakeCalc?.cancel();outTake1.power=0.0;outTake2.power=0.0
            runIntake?.cancel();intakeServo1.power=0.0
        }
        isSlowMode = gamepad1.cross
    }

    override fun stop() {
        outTakeCalc?.cancel()
        runIntake?.cancel()
        runDetections?.cancel()
    }

    suspend fun handleFiring() {
        val triggerPressed = gamepad1.right_trigger > 0.5

        if (triggerPressed && !lastTriggerPressed && !isDispensing) {
            depoCentered = true
            isDispensing = true
        }

        lastTriggerPressed = triggerPressed

        if (depoCentered && isDispensing) {
            centerDepo()
        }
    }
    suspend fun centerDepo() {
        follower.setMaxPower(0.18)
        val result: LLResult? = limelight.latestResult
        val fiducialResults = result?.fiducialResults
        val target = fiducialResults?.firstOrNull { it.fiducialId == AprilTagIds.BLUE_DEPO }

        if (target == null) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
            return
        }

        val xErrPx: Double = target.targetXPixels - (DepoCenter.CAM_WIDTH_PX / 2.0)

        if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)

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
    suspend fun reCenterDepo() {
        follower.setMaxPower(0.18)
        val result: LLResult? = limelight.latestResult
        val fiducialResults = result?.fiducialResults
        val target = fiducialResults?.firstOrNull { it.fiducialId == AprilTagIds.BLUE_DEPO }

        if (target == null) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
            return
        }

        val xErrPx: Double = target.targetXPixels - (DepoCenter.CAM_WIDTH_PX / 2.0)

        if (abs(xErrPx) <= DepoCenter.CENTER_DEADZONE) {
            follower.setTeleOpDrive(0.0, 0.0, 0.0, false)
            return
        }

        val rotationPower = clip(xErrPx * DepoCenter.KP_ROTATE, -0.3, 0.3)
        follower.setTeleOpDrive(0.0, 0.0, -rotationPower, false)
    }
    suspend fun executeDispensing() {
        val dispenseSequence = mutableListOf<Double>()

        // Map each slot to its firing position
        val slotToFirePosition = mapOf(
            0 to ServoPositions.FIRE_P1,
            1 to ServoPositions.FIRE_P2,
            2 to ServoPositions.FIRE_P3
        )

        // The order we want to dispense: slot 1 (index 1), slot 0 (index 0), slot 2 (index 2)
        // This matches your original sequence: FIRE_P2, FIRE_P1, FIRE_P3
        val dispenseOrder = listOf(1, 0, 2)

        for (slotIndex in dispenseOrder) {
            if (ord[slotIndex] != "N") {
                slotToFirePosition[slotIndex]?.let { dispenseSequence.add(it) }
            }
        }

        // Only dispense if we have at least one ball
        if (dispenseSequence.isNotEmpty()) {
            reCenterDepo()
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
    suspend fun executeDispenseSequence(positions: List<Double>) {
        positions.forEach { position ->
            bowlServo.position = position
            delay(Timing.BOWL_MOVE_DELAY)

            camServo.position = ServoPositions.CAM_OPEN
            delay(Timing.CAM_OPEN_DELAY)

            camServo.position = ServoPositions.CAM_CLOSED
            delay(Timing.CAM_CLOSE_DELAY)
        }
    }
    fun outTakePower() {
        val result: LLResult? = limelight.latestResult
        if (result == null || !result.isValid) {
            return
        }
        val fiducialResults = result.fiducialResults
        val target = fiducialResults.firstOrNull { it.fiducialId == AprilTagIds.BLUE_DEPO }
        if (target == null) {
            return
        }
        val targetY = target.targetYPixels
        var powerResult = 0.0002416*targetY+0.105
        DepoCenter.OUTTAKE_SPEED = powerResult
        outTake1.power = DepoCenter.OUTTAKE_SPEED
        outTake2.power = DepoCenter.OUTTAKE_SPEED
    }
    fun handleIntake() {
        val isFull: Boolean = if (gamepad1.circle) { true } else { ord.none { it == "N" } }
        if (isFull) {
            intakeServo1.power = ServoPositions.INTAKE_REVERSE
            bothLEDon()
        } else {
            intakeServo1.power = ServoPositions.INTAKE_ON
            bothLEDoff()
        }
    }
    fun initializeHardware() {
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
    fun setupVision() {
        limelight.setPollRateHz(100)
        limelight.pipelineSwitch(0)
        limelight.start()
    }
    fun bothLEDoff() {
        listOf(statusLEDR, statusLEDG).forEach {
            it.state=true
        }
    }
    fun bothLEDon() {
        listOf(statusLEDR, statusLEDG).forEach {
            it.state=false
        }
    }
    fun setupLED() {
        listOf(statusLEDR, statusLEDG).forEach {
            it.state=true
        }
    }
    fun setupMotorDirections() {
        listOf(outTake2, liftLeft)
            .forEach { it.direction = DcMotorSimple.Direction.REVERSE }
    }
    fun setupPIDFCoefficients() {
        listOf(outTake1, outTake2)
            .forEach { it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF) }
    }
    fun resetEncoders() {
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
    fun resetServos() {
        camServo.position = ServoPositions.CAM_CLOSED
        bowlServo.position = ServoPositions.LOAD_P1
    }
    fun initializePedroPathing() {
        panels = PanelsTelemetry.telemetry
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()
        ord = arrayOf("N", "N", "N")
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()
    }
    fun ensureVelocityMode() {
        if (!velocityModeInitialized) {
            outTake1.mode = DcMotor.RunMode.RUN_USING_ENCODER
            outTake2.mode = DcMotor.RunMode.RUN_USING_ENCODER
            velocityModeInitialized = true
        }
    }
    fun setMotorVelocityFromPseudoPower(motor: DcMotorEx, power: Double) {
        ensureVelocityMode()
        motor.velocity = powerToTicksPerSecond(motor, power)
    }
    fun powerToTicksPerSecond(motor: DcMotorEx, power: Double): Double {
        val clipped = max(-1.0, min(1.0, power))
        val maxRpm = motor.motorType.maxRPM
        val tpr = motor.motorType.ticksPerRev
        val maxTicksPerSec = (maxRpm * tpr) / 60.0
        return clipped * velocityPowerScale * maxTicksPerSec
    }
    fun clip(v: Double, min: Double, max: Double): Double {
        return max(min, min(max, v))
    }
    suspend fun handleDetections() {
        if (isDispensing) return
        val r = colorSensor.red();val g = colorSensor.green();val b = colorSensor.blue()

        // "Clear" => re-arm immediately (even during cooldown)
        if (g <= 80 && r <= 110) {
            isSeen = false
            return
        }

        // If we're still in cooldown, don't trigger a new advance yet
        val now = System.currentTimeMillis()
        if (now < Timing.nextDetectAllowedMs) return

        // Ball present and we're armed
        if (!isSeen) {
            val slot = nextSlot()
            if (slot != -1) {
                // (Optional) choose G vs P here; your current logic makes g>=110 also count as P first.
                val newOrd = ord.copyOf()
                newOrd[slot] = if (g >= 110) "G" else "P"
                ord = newOrd

                advanceBowl(slot)

                // start cooldown WITHOUT blocking the loop
                Timing.nextDetectAllowedMs = now + Timing.DETECTION_COOLDOWN
            }
            isSeen = true
        }
    }
    fun nextSlot(): Int {
        return when {
            ord[0] == "N" -> 0
            ord[1] == "N" -> 1
            ord[2] == "N" -> 2
            else -> -1 // Array is full
        }
    }
    fun advanceBowl(slot: Int) {
        bowlServo.position = when (slot) {
            0 -> ServoPositions.LOAD_P2
            1 -> ServoPositions.LOAD_P3
            2 -> ServoPositions.FIRE_P2
            else -> bowlServo.position // Stay in place if full
        }
    }
}
