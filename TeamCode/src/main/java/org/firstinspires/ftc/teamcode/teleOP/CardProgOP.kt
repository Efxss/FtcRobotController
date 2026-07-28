package org.firstinspires.ftc.teamcode.teleOP

import com.pedropathing.ivy.Command
import com.pedropathing.ivy.commands.Commands
import com.pedropathing.ivy.groups.Groups
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.customOpMode.OutReachOpMode
import org.firstinspires.ftc.teamcode.subSystems.TagSS
import org.firstinspires.ftc.teamcode.util.DriveUtil
import org.firstinspires.ftc.teamcode.util.VariableStateUtil
import java.util.function.BooleanSupplier

@TeleOp(name = "Card Programming TeleOP", group = "TeleOP")
class CardProgOP: OutReachOpMode() {
    lateinit var driveUtil: DriveUtil
    lateinit var handleTags: Command
    val movingPidf = PIDFCoefficients(10.0, 0.0, 0.05, 0.025)
    override fun onInit() {
        tagSS = TagSS(hardwareMap)
        driveUtil = DriveUtil(hardwareMap, 0.3, 1.0, movingPidf)
    }

    override fun onLoop() {
        getDebugUtil().showTempDebug(
            "Current Tag: ${tagSS.currentTag()}",
            "Tag List Data: ${tagSS.tagListDat()}",
            "Tag List Size: ${tagSS.tagListSize()}"
        )
        getDebugUtil().update(telemetry)
        while (VariableStateUtil.tagList.size > 6) VariableStateUtil.tagList.remove(6)
        val idCase = LinkedHashMap<BooleanSupplier, Command>()
        handleTags = Commands.branch(idCase)
        for (tags in VariableStateUtil.tagList) {
            idCase[BooleanSupplier {tags == 21}] = driveUtil.setDrivePowersForPositiveTicksCommand(-0.4,-0.4, -500)
            idCase[BooleanSupplier {tags == 22}] = driveUtil.setDrivePowersForPositiveTicksCommand(-0.4,-0.4, -500)
            idCase[BooleanSupplier {tags == 23}] = driveUtil.setDrivePowersForPositiveTicksCommand(-0.4,-0.4, -500)
            idCase[BooleanSupplier {tags == 24}] = driveUtil.setDrivePowersForPositiveTicksCommand(-0.4,-0.4, -500)
        }
        if (gamepad1.psWasReleased()) execCards().schedule()
    }
    fun execCards(): Command = Groups.parallel(handleTags)
}