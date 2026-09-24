package org.firstinspires.ftc.teamcode.config.util

import com.bylazar.field.FieldManager
import com.bylazar.field.PanelsField
import com.bylazar.field.Style
import com.pedropathing.follower.Follower
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path

/**
 * Drawing utility object for visualizing robot position on Panels Dashboard
 */
object DrawingUtil {
    private val panelsField: FieldManager = PanelsField.field
    fun drawPose(follower: Follower) {
        panelsField.setOffsets(PanelsField.presets.PEDRO_PATHING)
        panelsField.setStyle("red","blue", 2.0)
        panelsField.setStyle(Style(fill = "red", "blue", 2.0))
        panelsField.moveCursor(follower.pose().x(),follower.pose().y())
        panelsField.circle(r = 2.0)
        panelsField.line(x2 = 10.0, y2 = 10.0)
        panelsField.rect(w = 10.0, h = 5.0)
        panelsField.update()
    }
}