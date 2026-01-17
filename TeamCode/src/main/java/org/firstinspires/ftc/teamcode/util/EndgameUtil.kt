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

        if (leftPos > Config.EndGame.LIFTMAX || rightPos > Config.EndGame.LIFTMAX) {
            liftLeft.power = 0.0
            liftRight.power = 0.0
            return
        }

        if (leftPos < Config.EndGame.SLOWMODE || rightPos < Config.EndGame.SLOWMODE) {
            liftLeft.power = Config.EndGame.NORMALSPEED
            liftRight.power = Config.EndGame.NORMALSPEED
        } else {
            liftLeft.power = Config.EndGame.SLOWSPEED
            liftRight.power = Config.EndGame.SLOWSPEED
        }
    }
}
