package org.firstinspires.ftc.teamcode.outReach

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
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

@TeleOp(name="JabCo OutReach",group="OutReach")
class JabCoOutReach: OpMode() {
    var panels:TelemetryManager?=null
    val scope=CoroutineScope(Dispatchers.Default + SupervisorJob())
    var runDetections:Job?= null
    var rItke:Job?=null
    var rLL:Job?=null
    var rDispense:Job?=null
    val startPose=Pose(72.0, 8.375, Math.toRadians(0.0))
    lateinit var follower:Follower
    lateinit var pathTimer:Timer
    lateinit var actionTimer:Timer
    lateinit var opmodeTimer:Timer
    lateinit var outTake1:DcMotorEx
    lateinit var outTake2:DcMotorEx
    lateinit var liftLeft:DcMotorEx
    lateinit var liftRight:DcMotorEx
    lateinit var intakeServo1:CRServo
    lateinit var intakeServo2:CRServo
    lateinit var bowlServo:Servo
    lateinit var camServo:Servo
    lateinit var limelight:Limelight3A
    lateinit var colorSensor:ColorSensor
    lateinit var statusLEDR:DigitalChannel
    lateinit var statusLEDG:DigitalChannel
    @Volatile var ord=arrayOf("N", "N", "N")
    @Volatile var eord=arrayOf("N", "N", "N")
    @Volatile var intakeActive = false
    var pathState:Int=0
    var dispensingState=0
    var isCentering=false
    @Volatile var isDispensing=false
    val pidP = 150.0
    val pidI = 0.0
    val pidD = 0.0
    val pidF = 13.5
    var velocityModeInitialized = false
    var velocityPowerScale = 0.85
    var intake = 0
    var isSeen = false
    object ServoPositions {
        const val LOAD_P1 = 0.021
        const val LOAD_P2 = 0.087
        const val LOAD_P3 = 0.158
        const val FIRE_P1 = 0.128
        const val FIRE_P2 = 0.195
        const val FIRE_P3 = 0.058
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
        const val KP_ROTATE = 0.003
        var CENTER_DEADZONE = 13
        var OUTTAKE_SPEED = 0.20
    }
    object Timing {
        const val DISPENSE_INITIAL_DELAY = 100L
        const val BOWL_MOVE_DELAY = 2500L
        const val CAM_OPEN_DELAY = 1400L
        const val CAM_CLOSE_DELAY = 1700L
        const val DETECTION_COOLDOWN = 400L
        const val OUTTAKE_DELAY = 800L
        var nextDetectAllowedMs = 0L
    }
    override fun init() {
        initializeHardware()
        initializePedroPathing()
    }
    override fun start() {
        runDetections = scope.launch {
            while (isActive) {
                handleDetections()
                delay(3)
            }
        }
        rItke = scope.launch {
            while (isActive) {
                runItke()
                delay(5)
            }
        }
        rLL = scope.launch {
            while (isActive) {
                calcmtf()
                delay(25)
            }
        }
        rDispense = scope.launch {
            while (isActive) {
                if (gamepad1.triangleWasReleased() && !isDispensing) dispenseAll()
                delay(10)
            }
        }
    }
    override fun loop() {
        panels?.apply {
            debug("ClrSnsrR",colorSensor.red())
            debug("ClrSnsrG",colorSensor.green())
            debug("ClrSnsrB",colorSensor.blue())
            debug("ClrSnsrA",colorSensor.alpha())
            debug("ClrSnsrAr",colorSensor.argb())
            debug("Ord1", ord[0])
            debug("Ord2", ord[1])
            debug("Ord3", ord[2])
            debug("eOrd1", eord[0])
            debug("eOrd2", eord[1])
            debug("eOrd3", eord[2])
            debug("itkeP1", intakeServo1.power)
            debug("itkeP2", intakeServo2.power)
            debug("FirePlan", buildFirePlan()?.joinToString(",") ?: "N/A")
            debug("Dispensing", isDispensing)
        }
        panels?.update(telemetry)
    }

    override fun stop() {
        runDetections?.cancel()
        rItke?.cancel()
        rLL?.cancel()
        rDispense?.cancel()
        limelight.stop()
    }
    fun buildFirePlan(): Array<Int>? {
        val plan = arrayOf(-1, -1, -1)
        val usedSlots = mutableSetOf<Int>()
        for (motifIdx in 0..2) {
            val neededColor = eord[motifIdx]
            if (neededColor == "N") return null
            var found = false
            for (slot in 0..2) {
                if (slot !in usedSlots && ord[slot] == neededColor) {
                    plan[motifIdx] = slot
                    usedSlots.add(slot)
                    found = true
                    break
                }
            }
            if (!found) return null
        }
        return plan
    }
    suspend fun dispenseAll() {
        if (eord.any { it == "N" }) return
        if (ord.any { it == "N" }) return
        val plan = buildFirePlan() ?: return
        isDispensing = true
        try {
            for (motifIdx in 0..2) {
                val slot = plan[motifIdx]
                fireFromSlot(slot)
                val updatedOrd = ord.copyOf()
                updatedOrd[slot] = "N"
                ord = updatedOrd
                if (motifIdx < 2) delay(Timing.DISPENSE_INITIAL_DELAY)
            }
        } finally {
            isDispensing = false
            bowlServo.position = ServoPositions.LOAD_P1
        }
    }
    private suspend fun fireFromSlot(slot: Int) {
        val firePosition = when (slot) {
            0 -> ServoPositions.FIRE_P1
            1 -> ServoPositions.FIRE_P2
            2 -> ServoPositions.FIRE_P3
            else -> return
        }
        bowlServo.position = firePosition
        delay(Timing.BOWL_MOVE_DELAY)
        outTake1.power = DepoCenter.OUTTAKE_SPEED
        outTake2.power = DepoCenter.OUTTAKE_SPEED
        delay(Timing.OUTTAKE_DELAY)
        camServo.position = ServoPositions.CAM_OPEN
        delay(Timing.CAM_OPEN_DELAY)
        camServo.position = ServoPositions.CAM_CLOSED
        delay(Timing.CAM_CLOSE_DELAY)
        outTake1.power = 0.0
        outTake2.power = 0.0
    }
    suspend fun handleDetections() {
        if (isDispensing) return
        val r = colorSensor.red();val g = colorSensor.green();val b = colorSensor.blue()
        if (g <= 80 && r <= 110) {
            isSeen = false
            return
        }
        val now = System.currentTimeMillis()
        if (now < Timing.nextDetectAllowedMs) return
        if (!isSeen) {
            val slot = nextSlot()
            if (slot != -1) {
                val newOrd = ord.copyOf()
                newOrd[slot] = if (g >= 350) "G" else "P"
                ord = newOrd
                advanceBowl(slot)
                Timing.nextDetectAllowedMs = now + Timing.DETECTION_COOLDOWN
            }
            isSeen = true
        }
    }
    suspend fun runItke() {
        if (gamepad1.crossWasReleased()) intakeActive = true
        if (ord[2] != "N") intakeActive = false
        if (intakeActive) {
            intakeServo1.power = ServoPositions.INTAKE_ON
            intakeServo2.power = ServoPositions.INTAKE_ON
        } else {
            intakeServo1.power = ServoPositions.INTAKE_OFF
            intakeServo2.power = ServoPositions.INTAKE_OFF
        }
    }
    suspend fun calcmtf() {
        val result = limelight.latestResult
        if (result != null && result.isValid) {
            val detections = result.fiducialResults
            for (detection in detections) {
                when (detection.fiducialId) {
                    AprilTagIds.GPP_ORDER -> eord = arrayOf("G", "P", "P")
                    AprilTagIds.PGP_ORDER -> eord = arrayOf("P", "G", "P")
                    AprilTagIds.PPG_ORDER -> eord = arrayOf("P", "P", "G")
                }
            }
        }
    }
    fun nextSlot(): Int {
        return when {
            ord[0] == "N" -> 0
            ord[1] == "N" -> 1
            ord[2] == "N" -> 2
            else -> -1
        }
    }
    suspend fun advanceBowl(slot: Int) {
        delay(50)
        if (isDispensing) return
        bowlServo.position = when (slot) {
            0 -> ServoPositions.LOAD_P2
            1 -> ServoPositions.LOAD_P3
            2 -> ServoPositions.FIRE_P2
            else -> bowlServo.position
        }
    }
    fun initializeHardware() {
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
        statusLEDR = hardwareMap.get(DigitalChannel::class.java, "statusLEDR")
        statusLEDG = hardwareMap.get(DigitalChannel::class.java, "statusLEDG")
        statusLEDR.mode = DigitalChannel.Mode.OUTPUT
        statusLEDG.mode = DigitalChannel.Mode.OUTPUT
        setupMotorDirections()
        setupPIDFCoefficients()
        resetEncoders()
        resetServos()
        setupVision()
    }
    fun setupVision() {
        limelight.setPollRateHz(200)
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
        listOf(outTake2, liftLeft, intakeServo2)
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
}