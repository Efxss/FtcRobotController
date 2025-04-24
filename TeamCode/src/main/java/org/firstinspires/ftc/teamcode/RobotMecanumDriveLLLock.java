// [IMPORT] FTC
package org.firstinspires.ftc.teamcode;

// [IMPORT] REV
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

// [IMPORT] LIMELIGHT 3A
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name="Drive LL LOCK", group="TeleOp")
public class DriveLLLOCK extends OpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private boolean lockMode = false;
    private boolean lastLockModeState = false;
    private Limelight3A limelight;
    private double speedMult = 1;

    @Override
    public void init() {
        // [SETUP] Hardware Map
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");

        // [SETUP] Motor Config.
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // [SETUP] Limelight3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        // [DEBUG] ScriptStatus
        telemetry.addData("Status:", "Initialized");
        telemetry.addData("!WARNING!:", "Potential robot spazzing, as this is a test");
    }

    @Override
    public void loop() {
        // [INPUT] Lock Mode (BOOLEAN, 'A')
        if (gamepad1.a && !lastLockModeState) {
            lockMode = !lockMode;
        }
        lastLockModeState = gamepad1.a;

        // [INPUT] Drive (DOUBLE, Jstick)
        double drive  = gamepad1.left_stick_y * speedMult;  // Forward/Backward
        double strafe = -gamepad1.left_stick_x * speedMult; // Left/Right
        double rotate = gamepad1.left_trigger - gamepad1.right_trigger * speedMult; // Rotation

        // [SCRIPT] Lock Mode
        if (lockMode) {
            drive *= 0.5;
            strafe *= 0.5;
            rotate *= 0.5;
            double[] movementAdjustments = getAlignMovement();
            strafe += movementAdjustments[0];
            drive += movementAdjustments[1];
            rotate += movementAdjustments[2];
        }

        // [SCRIPT] Movement Calc (DRIVE: MECANUM)
        double frontLeftPower  = drive + strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backLeftPower   = drive - strafe + rotate;
        double backRightPower  = drive + strafe - rotate;

        // [OUTPUT] Motor Movement(s)
        frontLeft.setPower(frontLeftPower/2);
        frontRight.setPower(frontRightPower/2);
        backLeft.setPower(backLeftPower/2);
        backRight.setPower(backRightPower/2);

        // [DEBUG] LockModeStatus, MotorPowers (Tele)
        telemetry.addData("Lock Mode", lockMode ? "ON" : "OFF");
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.update();
    }

    @Override
    // [HELPER-END] Stop: Motors, Limelight
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        limelight.stop();
    }

    // [FUNCTION] Alignment Calc (Limelight) (WITH DEBUG: TELE)
    private double[] getAlignMovement() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // Left/Right Offset
            double ty = result.getTy(); // Up/Down Offset
            double ta = result.getTa(); // Target Area (Size)
            
            double strafeAdjust = tx * 0.02;  // Side/Side (X)
            double driveAdjust = (10 - ta) * 0.01; // Forward/Backward
            double rotateAdjust = tx * 0.015; // Rotate
            
            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            return new double[]{strafeAdjust, driveAdjust, rotateAdjust};
        } else {
            telemetry.addData("Limelight", "No Targets");
            return new double[]{0, 0, 0};
        }
    }
}
