package org.firstinspires.ftc.teamcode.teleOP

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import org.firstinspires.ftc.teamcode.customOpMode.OutReachOpMode
import org.firstinspires.ftc.teamcode.subSystems.CamSS
import org.firstinspires.ftc.teamcode.subSystems.SpinDexerSS
import org.firstinspires.ftc.teamcode.util.DriveUtil
import org.firstinspires.ftc.teamcode.util.FiringUtil

@TeleOp(name = "Robot Says TeleOP", group = "TeleOP")
class RobotSaysOP : OutReachOpMode() {
    lateinit var driveUtil: DriveUtil
    private lateinit var spinDexer : SpinDexerSS
    private lateinit var firing : FiringUtil
    private lateinit var cam : CamSS
    private val firingPidf = PIDFCoefficients(25.0, 0.0, 0.0, 13.5)
    val movingPidf = PIDFCoefficients(10.0, 0.0, 0.05, 0.025)
    override fun onInit() {
        driveUtil = DriveUtil(hardwareMap, 0.3, 1.0, movingPidf)
        cam = CamSS(hardwareMap, 0.55 , 0.0)
        spinDexer = SpinDexerSS(hardwareMap)
        firing = FiringUtil(hardwareMap, spinDexer, cam, 0.45, 1.0, firingPidf)
        cam.home()
        spinDexer.loadOne(true)
    }
    override fun onLoop() {
        if (gamepad1.dpad_up) {
            driveUtil.setDrivePowersForTicks(-0.4,-0.4, -500)
        } else if (gamepad1.dpadUpWasReleased()) driveUtil.resetTicks()
        if (gamepad1.dpad_down) {
            driveUtil.setDrivePowersForPositiveTicks(0.4,0.4, 500)
        } else if (gamepad1.dpadDownWasReleased()) driveUtil.resetTicks()
        if (gamepad1.dpad_left) {
            driveUtil.setDrivePowersForPositiveTicks(0.4,-0.4, 500)
        } else if (gamepad1.dpadLeftWasReleased()) driveUtil.resetTicks()
        if (gamepad1.dpad_right) {
            driveUtil.setDrivePowersForTicks(-0.4,0.4, -500)
        } else if (gamepad1.dpadRightWasReleased()) driveUtil.resetTicks()
        if (gamepad1.crossWasReleased()) {
            driveUtil.resetTicks()
        }
        firing.apply {
            startFiring(gamepad1.psWasReleased())
            update()
        }
    }
}