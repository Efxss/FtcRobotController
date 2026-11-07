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
            addLine("=== PedroPathing ===")
            addData("Follower Pose X", follower.pose().x())
            addData("Follower Pose Y", follower.pose().y())
            addData("Follower Heading", Math.toDegrees(follower.pose().heading()))
            addData("Follower Distance Remaining", follower.remainingDistance())
            addData("Follower IsBusy", follower.isBusy)
            addLine("")
            addLine("=== OpMode ===")
            addData("runtime", runtime)
            addData("Alliance", alliance.name)
        }
    }

    fun showAllDebugTeleop(
        follower: Follower,
        alliance: Robot.Alliance,
        runtime: Double,
        gamepad: Gamepad,
        fieldHalf: Robot.FieldHalf,
        firingSS: FiringSS,
        //volts: Double,
        intakeSS: IntakeSS,
        robot: Robot,
    ) {
        panels?.apply {
            addLine("=== PedroPathing ===")
            //addData("Follower Pose X", follower.pose().x())
            //addData("Follower Pose Y", follower.pose().y())
            //addData("Follower Heading", Math.toDegrees(follower.pose().heading()))
            //addData("Dist From Ref Pose", follower.pose().distance(robot.refPose))
            //addLine("")
            addLine("=== HardWare ===")
            addData("Intake Group Speed", robot.intakeMG.velocity)
            addData("Intake 1 Speed", robot.intakeM1.velocity)
            addData("Intake 1 Corrected Velocity", robot.intakeM1.correctedVelocity)
            addData("Intake 2 Speed", robot.intakeM2.velocity)
            addData("intake PIDF Point", robot.intakeMPIDF.setPoint)
            addData("intake PIDF Velocity Error", robot.intakeMPIDF.velocityError)
            addData("intake PIDF Position Error", robot.intakeMPIDF.positionError)
            addData("is Full", intakeSS.isFull())
            //addData("Shooter Speed", robot.fireM.velocity)
            //addData("Shooter Velocity", firingSS.getVelocity())
            //addData("Shooter PIDF Calc", firingSS.getPower())
            addLine("")
            addLine("=== Gamepad ===")
            //addData("Left Stick X", gamepad.left_stick_x)
            //addData("Left Stick Y", gamepad.left_stick_y)
            //addData("Right Stick X", gamepad.right_stick_x)
            addData("Left Bumper", gamepad.left_bumper)
            addData("Right Bumper", gamepad.right_bumper)
            addLine("")
            addLine("=== OpMode ===")
            //addData("millivolts", volts)
            //addData("runtime", runtime)
            //addData("Field Half", fieldHalf.name)
            //addData("Alliance", alliance.name)
        }
    }

    fun showInit() {
        panels?.debug("Init Started")
    }
    fun update(telemetry: org.firstinspires.ftc.robotcore.external.Telemetry) {
        panels?.update(telemetry)
    }
}