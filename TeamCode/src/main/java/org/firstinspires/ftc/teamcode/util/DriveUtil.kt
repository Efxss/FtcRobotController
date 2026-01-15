package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.Gamepad
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.pow
import kotlin.math.sqrt

object DriveUtil {

    data class DriveCommand(
        val forward: Double,
        val strafe: Double,
        val rotate: Double,
        val maxPower: Double,
        val fieldCentric: Boolean
    )

    private object Turn {
        const val DEADBAND = 0.05
        const val MAX_TURN = 0.8
    }

    private object Scale {
        const val DEADBAND = 0.06   // ignore tiny stick noise
        const val MIN_SCALE = 0.20  // minimum drive authority at low input
        const val EXPO = 1.6        // 1.0=linear, >1 softer near center
    }

    fun commandFrom(
        gamepad: Gamepad,
        isDispensing: Boolean,
        slowMode: Boolean,
        normalCap: Double = TeleOpConfig.DriveCaps.NORMAL_MAX,
        slowCap: Double = TeleOpConfig.DriveCaps.SLOW_MAX,
        fieldCentric: Boolean = true
    ): DriveCommand {
        if (isDispensing) {
            return DriveCommand(0.0, 0.0, 0.0, 0.0, false)
        }

        val forward = -gamepad.left_stick_y.toDouble()
        val strafe = gamepad.right_stick_x.toDouble()

        // triggers = analog turning
        val turnInput = (gamepad.left_trigger - gamepad.right_trigger).toDouble()
        val rotate = if (abs(turnInput) < Turn.DEADBAND) 0.0 else turnInput * Turn.MAX_TURN

        val baseMax = if (slowMode) slowCap else normalCap
        val demandScale = computeDemandScale(forward, strafe, rotate)

        return DriveCommand(
            forward = forward,
            strafe = strafe,
            rotate = rotate,
            maxPower = baseMax * demandScale,
            fieldCentric = fieldCentric
        )
    }

    private fun computeDemandScale(forward: Double, strafe: Double, rotate: Double): Double {
        val transMag = sqrt(forward * forward + strafe * strafe).coerceIn(0.0, 1.0)
        val rotMag = abs(rotate).coerceIn(0.0, 1.0)
        var mag = max(transMag, rotMag)

        if (mag < Scale.DEADBAND) mag = 0.0

        val norm =
            if (mag == 0.0) 0.0
            else ((mag - Scale.DEADBAND) / (1.0 - Scale.DEADBAND)).coerceIn(0.0, 1.0)

        val curved = norm.pow(Scale.EXPO)

        return (Scale.MIN_SCALE + (1.0 - Scale.MIN_SCALE) * curved).coerceIn(0.0, 1.0)
    }
}
