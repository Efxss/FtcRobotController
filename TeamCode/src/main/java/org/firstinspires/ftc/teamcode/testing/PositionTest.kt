package org.firstinspires.ftc.teamcode.testing

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.util.Timer
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.util.Drawing

@Disabled
//@TeleOp(name = "Position Test", group = "Test")
class PositionTest : OpMode() {
    var panels: TelemetryManager? = null
    @Volatile lateinit var follower: Follower
    @Volatile lateinit var pathTimer: Timer
    @Volatile lateinit var actionTimer: Timer
    @Volatile lateinit var opmodeTimer: Timer
    var pathState: Int = 0
    var timerState = false
    private val startPose = Pose(72.0,0.0,Math.toRadians(90.0))
    override fun init() {
        initializePedroPathing()
    }
    override fun init_loop() {
        follower.update()
        Drawing.drawOnlyCurrent(follower)
    }
    override fun loop() {
        follower.update()
        Drawing.drawDebug(follower)
        panels?.debug("X", follower.pose.x)
        panels?.debug("Y", follower.pose.y)
        panels?.debug("H", Math.toDegrees(follower.pose.heading))
        panels?.debug("Timer", pathTimer.elapsedTimeSeconds)
        panels?.debug("RunTime", runtime)
        panels?.update(telemetry)
    }
    fun initializePedroPathing() {
        panels = PanelsTelemetry.telemetry
        pathTimer = Timer()
        actionTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer.resetTimer()
        val otos = hardwareMap.get(SparkFunOTOS::class.java, "otos")
        otos.calibrateImu()
        otos.resetTracking()
        follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)
        follower.activateAllPIDFs()
        Drawing.init()
    }
}