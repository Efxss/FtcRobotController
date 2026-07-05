package org.firstinspires.ftc.teamcode.config.subSystem

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands.instant
import com.pedropathing.ivy.groups.Groups.sequential
import org.firstinspires.ftc.teamcode.config.util.VariableStateUtil

class FiringSS {
    fun execFiring(sweepSS: SweepSS): Command {
        return sequential(
            rampFire(),
            sweepSS.execSweepServo(),
            rampIntake()
        )
    }
    private fun rampFire(): Command { return instant { VariableStateUtil.rampState = RampSS.STATE.FIRE } }
    private fun rampIntake(): Command { return instant { VariableStateUtil.rampState = RampSS.STATE.INTAKE } }
}