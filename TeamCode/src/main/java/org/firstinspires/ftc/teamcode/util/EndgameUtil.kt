package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.DcMotorEx

object EndgameUtil {

    /**
     * Keeps the same logic shape you had:
     * - Below SLOWMODE => full speed
     * - Above SLOWMODE => slow speed
     * - Above LIFTMAX => stop
     *
     * Note: your original else-if ordering means the LIFTMAX stop may never be reached
     * depending on values; we preserve intent by checking LIFTMAX first.
     */
    fun applyLiftLogic(liftLeft: DcMotorEx, liftRight: DcMotorEx) {
        val leftPos = liftLeft.currentPosition
        val rightPos = liftRight.currentPosition

        if (leftPos > TeleOpConfig.EndGame.LIFTMAX || rightPos > TeleOpConfig.EndGame.LIFTMAX) {
            liftLeft.power = 0.0
            liftRight.power = 0.0
            return
        }

        if (leftPos < TeleOpConfig.EndGame.SLOWMODE || rightPos < TeleOpConfig.EndGame.SLOWMODE) {
            liftLeft.power = TeleOpConfig.EndGame.NORMALSPEED
            liftRight.power = TeleOpConfig.EndGame.NORMALSPEED
        } else {
            liftLeft.power = TeleOpConfig.EndGame.SLOWSPEED
            liftRight.power = TeleOpConfig.EndGame.SLOWSPEED
        }
    }
}
