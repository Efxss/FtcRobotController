package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.subSystems.CamSS
import org.firstinspires.ftc.teamcode.subSystems.SpinDexerSS
import java.lang.Thread.sleep
import kotlin.math.max
import kotlin.math.min

class FiringUtil(
    hardwareMap: HardwareMap
) {
    private val flyWheel: DcMotorEx = hardwareMap.get(DcMotorEx::class.java, "flyWheel")
    private var spinDexer: SpinDexerSS = SpinDexerSS(hardwareMap)
    private var cam: CamSS = CamSS(hardwareMap)
    val p = 150.0
    val i = 0.0
    val d = 0.0
    val f = 13.5
    var velocityModeInitialized = false
    var velocityPowerScale = 0.85

    init {
        ensureVelocityMode()
        setPIDF()
    }

    fun executeFiring(button: Boolean) {
        if (!button) return

        setMotorVelocityFromPseudoPower(flyWheel, 1.0)

        fireSequence(
            { spinDexer.fireTwo() },
            { spinDexer.fireThree() },
            { spinDexer.fireOne() }
        )

        spinDexer.loadOne(true)
        setMotorVelocityFromPseudoPower(flyWheel, 0.0)
    }

    fun fireSequence(vararg steps: () -> Unit) {
        for (step in steps) {
            step()
            sleep(1000)
            cam.fireCam()
        }
    }

    fun setPIDF() {
        listOf(flyWheel)
            .forEach { it.setVelocityPIDFCoefficients(p, i, d, f) }
    }

    fun ensureVelocityMode() {
        if (!velocityModeInitialized) {
            flyWheel.mode = DcMotor.RunMode.RUN_USING_ENCODER
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
}