package org.firstinspires.ftc.teamcode.opmode.auto.blue

import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.groups.Groups
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode

@Autonomous(group = "Auto", name = "Auto")
class BlueAuto : AutoOpMode() {
    override val alliance = Robot.Alliance.BLUE
    override val opmode = Robot.OpMode.AUTO

    override fun onInit() {
        robot.initPedro(hardwareMap, opmode)
    }

    override fun onStart() {
        intakeSS.runIntake.schedule()
        Scheduler.schedule(
            Groups.sequential(
                runAuto()
            )
        )
    }

    override fun onLoop() {
        robot.follower.update()
    }
}