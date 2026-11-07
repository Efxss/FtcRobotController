package org.firstinspires.ftc.teamcode.config.util

import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.config.Robot
import org.firstinspires.ftc.teamcode.config.subSystem.FiringSS
import org.firstinspires.ftc.teamcode.config.subSystem.IntakeSS

class PanelsDebugUtil(
    private val panels: TelemetryManager?
) {
    fun showAllDebugAuto(
        follower: Follower,
        alliance: Robot.Alliance,
        runtime: Double
    ) {
        panels?.apply {
            debug("=== PedroPathing ===")
            debug("Follower Pose X", follower.pose().x())
            debug("Follower Pose Y", follower.pose().y())
            debug("Follower Heading", Math.toDegrees(follower.pose().heading()))
            debug("Follower Distance Remaining", follower.remainingDistance())
            debug("Follower IsBusy", follower.isBusy)
            debug("")
            debug("=== OpMode ===")
            debug("runtime", runtime)
            debug("Alliance", alliance.name)
        }
    }

    fun showAllDebugTeleop(
        follower: Follower,
        alliance: Robot.Alliance,
        runtime: Double,
        gamepad: Gamepad,
        fieldHalf: Robot.FieldHalf,
        firingSS: FiringSS,
        volts: Double,
        intakeSS: IntakeSS,
        robot: Robot,
    ) {
        panels?.apply {
            debug("=== PedroPathing ===")
            //debug("Follower Pose X", follower.pose().x())
            //debug("Follower Pose Y", follower.pose().y())
            //debug("Follower Heading", Math.toDegrees(follower.pose().heading()))
            //debug("Dist From Ref Pose", follower.pose().distance(robot.refPose))
            //debug("")
            debug("=== HardWare ===")
            addData("Intake Group Speed", robot.intakeMG.velocity)
            addData("Intake 1 Speed", robot.intakeM1.velocity)
            addData("Intake 1 Corrected Velocity", robot.intakeM1.correctedVelocity)
            addData("Intake 2 Speed", robot.intakeM2.velocity)
            addData("intake PIDF Point", robot.intakeMPIDF.setPoint)
            addData("intake PIDF Velocity Error", robot.intakeMPIDF.velocityError)
            addData("intake PIDF Position Error", robot.intakeMPIDF.positionError)
            addData("is Full", intakeSS.isFull(volts))
            //addData("Shooter Speed", robot.fireM.velocity)
            //addData("Shooter Velocity", firingSS.getVelocity())
            //addData("Shooter PIDF Calc", firingSS.getPower())
            debug("")
            debug("=== Gamepad ===")
            //debug("Left Stick X", gamepad.left_stick_x)
            //debug("Left Stick Y", gamepad.left_stick_y)
            //debug("Right Stick X", gamepad.right_stick_x)
            debug("Left Bumper", gamepad.left_bumper)
            debug("Right Bumper", gamepad.right_bumper)
            debug("")
            debug("=== OpMode ===")
            //debug("volts/millivolts", volts)
            //debug("runtime", runtime)
            //debug("Field Half", fieldHalf.name)
            //debug("Alliance", alliance.name)
        }
    }

    fun showInit() {
        panels?.debug("Init Started")
    }
    fun update(telemetry: org.firstinspires.ftc.robotcore.external.Telemetry) {
        panels?.update(telemetry)
    }
}