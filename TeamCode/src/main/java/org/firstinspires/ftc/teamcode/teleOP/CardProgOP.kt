package org.firstinspires.ftc.teamcode.teleOP

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.customOpMode.OutReachOpMode
import org.firstinspires.ftc.teamcode.subSystems.TagSS
import org.firstinspires.ftc.teamcode.util.DriveUtil
import org.firstinspires.ftc.teamcode.util.VariableStateUtil

@TeleOp(name = "Card Programming TeleOP", group = "TeleOP")
class CardProgOP: OutReachOpMode() {
    lateinit var driveUtil: DriveUtil
    lateinit var handleTags: Command
    val drivePower = 0.2
    val movingPidf = PIDFCoefficients(10.0, 0.0, 0.05, 0.025)
    override fun onInit() {
        tagSS = TagSS(hardwareMap)
        driveUtil = DriveUtil(hardwareMap, 0.3, 1.0, movingPidf)
    }
    override fun onLoop() {
        Scheduler.execute()
        tagSS.update(runtime, ledss)
        getDebugUtil().showTempDebug(tagSS, driveUtil)
        getDebugUtil().update(telemetry)
        if (gamepad1.psWasReleased()) {
            execCards().schedule()
            if (!Scheduler.isRunning(execCards())) {
               VariableStateUtil.tagList.clear()
               tagSS.clearList().schedule()
            }
        }
    }
    fun cardCommand(id: Int): Command? = when (id) {
        21 -> driveUtil.setDrivePowerForTicksCommand(-drivePower, -drivePower, -250) // Up
        22 -> driveUtil.setDrivePowerForTicksCommand(-drivePower, drivePower, -550) // Right
        23 -> driveUtil.setDrivePowerForPositiveTicksCommand(drivePower, -drivePower, 500) // Left
        24 -> driveUtil.setDrivePowerForPositiveTicksCommand(drivePower, drivePower, 250) // Down
        else -> null
    }
    fun execCards(): Command = Groups.sequential(*VariableStateUtil.tagList.mapNotNull { cardCommand(it) }.toTypedArray()).then(Commands.instant { tagSS.clearList() })
    override fun onStop() {
        VariableStateUtil.tagList.clear()
        tagSS.clearList().schedule()
        driveUtil.resetTicks()
        Scheduler.reset()
    }
}