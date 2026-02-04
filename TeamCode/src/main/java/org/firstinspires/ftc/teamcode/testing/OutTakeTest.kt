package org.firstinspires.ftc.teamcode.testing

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.math.max
import kotlin.math.min

@Disabled
//@Configurable
//@TeleOp(name = "OutTake Test", group = "Test")
class OutTakeTest : OpMode() {
    var panels: TelemetryManager? = null
    private lateinit var outTake1: DcMotorEx
    private lateinit var outTake2: DcMotorEx
    private var velocityModeInitialized = false
    private var velocityPowerScale = 1.0
    companion object {
        @JvmField
        var pidP = 150.0
        var pidI = 0.0
        var pidD = 0.0
        var pidF = 13.5
        var power = 0.27
    }
    override fun init() {
        panels = PanelsTelemetry.telemetry
        outTake1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        outTake2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        resetEncoders()
    }

    override fun loop() {
        listOf(outTake1, outTake2)
            .forEach { it.setVelocityPIDFCoefficients(pidP, pidI, pidD, pidF) }
        setMotorVelocityFromPseudoPower(outTake1, power)
        setMotorVelocityFromPseudoPower(outTake2, power)
        panels?.apply {
            debug("OutTake1 velocity", outTake1.velocity)
            debug("OutTake2 velocity", outTake2.velocity)
            debug("OutTake1 power", outTake1.power)
            debug("OutTake2 power", outTake2.power)
            debug("P", pidP)
            debug("I", pidI)
            debug("D", pidD)
            debug("F", pidF)
            update(telemetry)
        }
    }

    private fun resetEncoders() {
        listOf(outTake1, outTake2).forEach { motor ->
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
        outTake2.direction = DcMotorSimple.Direction.REVERSE
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
}
