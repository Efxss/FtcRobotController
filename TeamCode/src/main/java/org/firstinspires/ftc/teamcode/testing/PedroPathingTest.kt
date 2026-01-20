package org.firstinspires.ftc.teamcode.testing

import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@Disabled
class PedroPathingTest : OpMode() {
    var panels: TelemetryManager? = null
    private val startPose    = Pose(123.0, 123.0, Math.toRadians(36.0))
    private val scorePose    = Pose(91.5, 91.5, Math.toRadians(45.0))
    private lateinit var preLoadScore: PathChain
    @Volatile lateinit var follower: Follower
    @Volatile lateinit var pathTimer: Timer
    @Volatile lateinit var actionTimer: Timer
    @Volatile lateinit var opmodeTimer: Timer
    @set:JvmName("PathStateVar")
    var pathState: Int = 0
    var timerState = false
    override fun init() {
        initializePedroPathing()
    }

    override fun loop() {
        follower.update()
        autonomousPathUpdate()
        panels?.debug("hi")
        panels?.update(telemetry)
    }
    fun autonomousPathUpdate() {
        val notBusy = !follower.isBusy
        when (pathState) {
            0 -> {
                if (notBusy && !timerState) {
                    pathTimer.resetTimer()
                    timerState = true
                    follower.followPath(preLoadScore, false)
                    setPathState(1)
                }
            }
        }
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

        buildPaths()
    }
    fun buildPaths() {
        preLoadScore = follower.pathBuilder()
            .addPath(BezierCurve(startPose, scorePose))
            .setLinearHeadingInterpolation(startPose.heading, scorePose.heading)
            .build()
    }
    @JvmName("SetPathStateFunction")
    private fun setPathState(pState: Int) {
        pathState = pState
        pathTimer.resetTimer()
        timerState = false
    }
}