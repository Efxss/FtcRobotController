package org.firstinspires.ftc.teamcode.OldCode

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import kotlin.concurrent.Volatile
import kotlin.math.max
import kotlin.math.min

@Configurable
@TeleOp
class OutTake : OpMode() {
    private var panels: TelemetryManager? = null
    private lateinit var motor1: DcMotorEx
    private lateinit var motor2: DcMotorEx

    //endregion
    @Volatile
    var velocityModeInitialized = false
    var velocityPowerScale = 0.95
    companion object {
        @JvmField
        var power1 = 0.toDouble()
        var power2 = 0.toDouble()
        var p = 10.toDouble()
        var i = 3.toDouble()
        var d = 2.toDouble()
        var f = 8.toDouble()
    }

    override fun init() {
        motor1 = hardwareMap.get(DcMotorEx::class.java, "outTake1")
        motor2 = hardwareMap.get(DcMotorEx::class.java, "outTake2")
        motor2.direction = DcMotorSimple.Direction.REVERSE
        panels = PanelsTelemetry.telemetry
    }

    override fun loop() {
        motor1.setVelocityPIDFCoefficients(p, i, d, f)
        motor2.setVelocityPIDFCoefficients(p, i, d, f)
        setMotorVelocityFromPseudoPower(motor1, power1) // 0.33
        setMotorVelocityFromPseudoPower(motor2, power2) // 0.33
        panels?.addData("Power1", power1)
        panels?.addData("Power2", power2)
        panels?.addData("Real Motor 1 Power", motor1.power)
        panels?.addData("Real Motor 2 Power", motor2.power)
        panels?.addData("Real Motor 1 velocity", motor1.velocity)
        panels?.addData("Real Motor 2 velocity", motor2.velocity)
        panels?.update()
    }

    private fun ensureVelocityMode() {
        if (!velocityModeInitialized) {
            motor1.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor2.mode = DcMotor.RunMode.RUN_USING_ENCODER
            velocityModeInitialized = true
        }
    }

    private fun powerToTicksPerSecond(motor: DcMotorEx, power: Double): Double {
        val clipped = max(-1.0, min(1.0, power))
        val maxRpm = motor.motorType.maxRPM // no-load
        val tpr = motor.motorType.ticksPerRev
        val maxTicksPerSec = (maxRpm * tpr) / 60.0
        return clipped * velocityPowerScale * maxTicksPerSec
    }

    private fun setMotorVelocityFromPseudoPower(motor: DcMotorEx, power: Double) {
        ensureVelocityMode()
        val tps = powerToTicksPerSecond(motor, power)
        motor.velocity = tps
    }
}