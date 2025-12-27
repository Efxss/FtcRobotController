package org.firstinspires.ftc.teamcode.testing

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "Cam Test", group = "Test")
@Configurable
class CamTest : OpMode() {
    private lateinit var outTake1:  DcMotorEx
    private lateinit var outTake2:  DcMotorEx
    private lateinit var camServo: Servo
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
        const val CAM_OPEN = 0.5
        const val CAM_CLOSED = 0.255

        const val INTAKE_ON = 1.0
        const val INTAKE_REVERSE = -1.0
        const val INTAKE_OFF = 0.0
    }
    private val pidP = 150.0
    private val pidI = 0.0
    private val pidD = 0.0
    private val pidF = 13.5
    companion object {
        @JvmField
        var power = 0.2
        var pos = 0.255
    }
    override fun init() {
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        camServo = hardwareMap.get(Servo::class.java, "camServo")
        outTake2.direction = DcMotorSimple.Direction.REVERSE
        camServo.position = ServoPositions.CAM_CLOSED
        listOf(outTake1, outTake2).forEach { it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF) }
    }

    override fun loop() {
        camServo.position = pos
        outTake1.power = power
        outTake2.power = power
    }
}