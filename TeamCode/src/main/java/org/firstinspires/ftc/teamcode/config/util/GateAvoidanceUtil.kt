package org.firstinspires.ftc.teamcode.config.util

import com.pedropathing.geometry.Pose
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

object GateAvoidanceUtil {

    var blueGatePose : Pose = Pose(7.1, 69.9, Math.toRadians(-180.0))

    var redGatePose : Pose = Pose(138.0, 69.9, Math.toRadians(180.0))

    var deadzone : Double = 12.0

    var lastDistanceToOpposingGate : Double = Double.POSITIVE_INFINITY
        private set

    var lastWasClamped : Boolean = false
        private set

    fun getOpposingGatePose(alliance : Alliance) : Pose = when (alliance) {
        Alliance.RED -> blueGatePose
        Alliance.BLUE -> redGatePose
    }
    fun applyAvoidance(
        forward : Double,
        strafe : Double,
        robotPose : Pose,
        alliance : Alliance
    ) : Pair<Double, Double> {
        val gate = getOpposingGatePose(alliance)
        val dx = robotPose.x - gate.x
        val dy = robotPose.y - gate.y
        val dist = hypot(dx, dy)
        lastDistanceToOpposingGate = dist
        if (dist >= deadzone || dist == 0.0) {
            lastWasClamped = false
            return forward to strafe
        }
        val h = robotPose.heading
        val cosH = cos(h)
        val sinH = sin(h)
        val awayX =  cosH * dx + sinH * dy
        val awayY = -sinH * dx + cosH * dy
        val nx = awayX / dist
        val ny = awayY / dist
        val inwardComponent = forward * -nx + strafe * -ny
        if (inwardComponent <= 0.0) {
            lastWasClamped = false
            return forward to strafe
        }
        val newForward = forward - inwardComponent * -nx
        val newStrafe  = strafe  - inwardComponent * -ny
        lastWasClamped = true
        return newForward to newStrafe
    }
}