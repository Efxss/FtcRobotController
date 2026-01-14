package org.firstinspires.ftc.teamcode.util

import com.bylazar.field.FieldManager
import com.bylazar.field.PanelsField
import com.bylazar.field.Style
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.PoseHistory

/**
 * Drawing utility object for visualizing robot position on Panels Dashboard
 */
object Drawing {
    const val ROBOT_RADIUS = 9.0
    private val panelsField: FieldManager = PanelsField.field

    // Styles for drawing
    private val robotStyle = Style("", "#4CAF50", 0.75)   // Green - current robot
    private val pathStyle = Style("", "#3F51B5", 0.75)    // Blue - path
    private val historyStyle = Style("", "#FF9800", 0.5)  // Orange - history trail

    fun init() {
        panelsField.setOffsets(PanelsField.presets.PEDRO_PATHING)
    }

    fun drawRobot(pose: Pose?, style: Style = robotStyle) {
        if (pose == null || pose.x.isNaN() || pose.y.isNaN() || pose.heading.isNaN()) return

        panelsField.setStyle(style)
        panelsField.moveCursor(pose.x, pose.y)
        panelsField.circle(ROBOT_RADIUS)

        // Draw heading line
        val v = pose.headingAsUnitVector
        v.magnitude *= ROBOT_RADIUS
        val x1 = pose.x + v.xComponent / 2
        val y1 = pose.y + v.yComponent / 2
        val x2 = pose.x + v.xComponent
        val y2 = pose.y + v.yComponent

        panelsField.setStyle(style)
        panelsField.moveCursor(x1, y1)
        panelsField.line(x2, y2)
    }

    fun drawPath(path: Path, style: Style = pathStyle) {
        val points = path.panelsDrawingPoints

        // Handle NaN values
        for (i in points[0].indices) {
            for (j in points.indices) {
                if (points[j][i].isNaN()) {
                    points[j][i] = 0.0
                }
            }
        }

        panelsField.setStyle(style)
        panelsField.moveCursor(points[0][0], points[0][1])
        panelsField.line(points[1][0], points[1][1])
    }

    fun drawPath(pathChain: PathChain, style: Style = pathStyle) {
        for (i in 0 until pathChain.size()) {
            drawPath(pathChain.getPath(i), style)
        }
    }

    fun drawPoseHistory(poseHistory: PoseHistory, style: Style = historyStyle) {
        panelsField.setStyle(style)

        val xPositions = poseHistory.xPositionsArray
        val yPositions = poseHistory.yPositionsArray
        val size = xPositions.size

        for (i in 0 until size - 1) {
            panelsField.moveCursor(xPositions[i], yPositions[i])
            panelsField.line(xPositions[i + 1], yPositions[i + 1])
        }
    }

    fun drawDebug(follower: Follower) {
        // Draw current path if available
        follower.currentPath?.let { path ->
            drawPath(path, pathStyle)

            // Draw closest point on path (target position)
            val closestPoint = follower.getPointFromPath(path.closestPointTValue)
            val headingGoal = path.getHeadingGoal(path.closestPointTValue)
            drawRobot(Pose(closestPoint.x, closestPoint.y, headingGoal), pathStyle)
        }

        // Draw pose history trail
        drawPoseHistory(follower.poseHistory, historyStyle)

        // Draw current robot position
        drawRobot(follower.pose, robotStyle)

        sendPacket()
    }

    fun drawOnlyCurrent(follower: Follower) {
        drawRobot(follower.pose)
        sendPacket()
    }

    fun sendPacket() {
        panelsField.update()
    }
}