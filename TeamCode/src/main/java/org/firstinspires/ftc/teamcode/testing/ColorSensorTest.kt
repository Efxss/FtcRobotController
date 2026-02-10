package org.firstinspires.ftc.teamcode.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.Servo
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay

@Configurable
@TeleOp
class ColorSensorTest : OpMode() {
    companion object {
        var shouldFire = false
    }
    var panels: TelemetryManager? = null
    val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
    var runDetections: Job?   = null
    var runIntake: Job?       = null
    var shoot: Job?           = null
    val dispenseSequence = mutableListOf<Double>()
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
    object Timing {
        const val DISPENSE_INITIAL_DELAY = 100L
        const val BOWL_MOVE_DELAY = 250L
        const val CAM_OPEN_DELAY = 140L
        const val CAM_CLOSE_DELAY = 170L
        const val DETECTION_COOLDOWN = 400L
        const val OUTTAKE_DELAY = 800L
        var nextDetectAllowedMs = 0L
    }
    lateinit var statusLEDR: DigitalChannel
    lateinit var statusLEDG:DigitalChannel
    lateinit var colorSensor: ColorSensor
    lateinit var intakeServo1: CRServo
    lateinit var outTake1:  DcMotorEx
    lateinit var outTake2: DcMotorEx
    lateinit var bowlServo: Servo
    lateinit var camServo: Servo
    @Volatile var ord = arrayOf("N", "N", "N")
    @Volatile var isDispensing = false
    @Volatile var isSeen = false
    override fun init() {
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        intakeServo1 = hardwareMap.get(CRServo::class.java, "intakeServo1")
        bowlServo = hardwareMap.get(Servo::class.java, "bowlServo")
        camServo = hardwareMap.get(Servo::class.java, "camServo")
        colorSensor = hardwareMap.get(ColorSensor::class.java, "ColorSensor")
        statusLEDR = hardwareMap.get(DigitalChannel::class.java, "statusLEDR")
        statusLEDG = hardwareMap.get(DigitalChannel::class.java, "statusLEDG")
        panels = PanelsTelemetry.telemetry
        bowlServo.position = ServoPositions.LOAD_P1
        setupMotorDirections()
        setupLED()
    }

    /* override fun start() {
        runDetections = scope.launch {
            while (isActive) {
                handleDetections()
                delay(3)
            }
        }
        runIntake = scope.launch {
            while (isActive) {
                handleIntake()
                delay(5)
            }
        }
        shoot = scope.launch { 
            while (isActive) {
                if (shouldFire) executeDispensing()
                delay(3)
            }
        }
    }*/
    override fun loop() {
        /*listOf(outTake1, outTake2).forEach { motor ->
            motor.power = 0.15
        }*/
        val r = colorSensor.red();val g = colorSensor.green();val b = colorSensor.blue()
        panels?.debug("Red", r, "Green", g, "Blue", b)
        //panels?.debug("Order 1", ord[0], "Order 2", ord[1], "Order 3", ord[2])
        panels?.update(telemetry)
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
    suspend fun handleDetections() {
        if (isDispensing) return
        val r = colorSensor.red();val g = colorSensor.green();val b = colorSensor.blue()
        if (g <= 80 && b <= 110) {
            isSeen = false
            return
        }
        val now = System.currentTimeMillis()
        if (now < Timing.nextDetectAllowedMs) return
        if (!isSeen) {
            val slot = nextSlot()
            if (slot != -1) {
                ord[slot] = if (g >= 110) "G" else "P"
                advanceBowl(slot)

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
            else -> -1 
        }
    }
    fun advanceBowl(slot: Int) {
        bowlServo.position = when (slot) {
            0 -> ServoPositions.LOAD_P2
            1 -> ServoPositions.LOAD_P3
            2 -> ServoPositions.FIRE_P2
            else -> bowlServo.position
        }
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
        listOf(outTake2).forEach { it.direction = DcMotorSimple.Direction.REVERSE }
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
    suspend fun executeDispensing() {
        val dispenseSequence = mutableListOf<Double>()
        val slotToFirePosition = mapOf(
            0 to ServoPositions.FIRE_P1,
            1 to ServoPositions.FIRE_P2,
            2 to ServoPositions.FIRE_P3
        )
        val dispenseOrder = listOf(1, 0, 2)
        for (slotIndex in dispenseOrder) {
            if (ord[slotIndex] != "N") {
                slotToFirePosition[slotIndex]?.let { dispenseSequence.add(it) }
            }
        }
        if (dispenseSequence.isNotEmpty()) {
            executeDispenseSequence(dispenseSequence)
        }
        delay(100)
        bowlServo.position = ServoPositions.LOAD_P1
        delay(100)
        shouldFire = false
        ord = arrayOf("N", "N", "N")
    }
}