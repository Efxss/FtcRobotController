package org.firstinspires.ftc.teamcode.opmode.auto.blue

import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.config.customOpMode.AutoOpMode
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.config.util.Alliance
import org.firstinspires.ftc.teamcode.config.util.AutoPoseUtil

@Autonomous(group = "Blue Auto", name = "Auto Pod Path Test Blue")
class BlueAuto : AutoOpMode() {
    override val alliance = Alliance.BLUE
    override fun onInit() {
        initializePedroPathing()
    }

    override fun onStart() {
        Scheduler.schedule(AutoPoseUtil.allSpikeAutoBlue())
    }

    override fun onLoop() {
        follower.update()
        Scheduler.execute()
    }

    fun initializePedroPathing() {
        Scheduler.reset()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(AutoPoseUtil.startPoseBlueDepoPose)
        AutoPoseUtil.follower = follower
    }
}