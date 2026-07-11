package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.waitMs
import com.pedropathing.ivy.groups.Groups.sequential

class FiringSS {
    fun execFiring(sweepSS: SweepSS, rampSS: RampSS, pushServoSS: PushServoSS): Command {
        return sequential(
            pushServoSS.runPush,
            rampSS.rampFire(),
            waitMs(3000.0),
            sweepSS.execSweepServo(),
            waitMs(3000.0),
            rampSS.rampIntake(),
            pushServoSS.stopPush
        )
    }
}