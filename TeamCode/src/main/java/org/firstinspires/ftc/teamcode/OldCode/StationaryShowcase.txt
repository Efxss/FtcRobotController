package org.firstinspires.ftc.teamcode.OldCode;

import static org.firstinspires.ftc.teamcode.OldCode.Utils.sleep;

import android.annotation.SuppressLint;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

import java.util.concurrent.ThreadLocalRandom;


@Disabled
@TeleOp
public class StationaryShowcase extends ThreadOpMode {

    //region GLOBAL VARIABLES

    // Hardware Variables:
    private Robot robot;
    private ColorSensor colorSensor;
    private CRServo IntakeServo1, IntakeServo2; private Servo CamServoPan, CamServoTilt;
    DcMotorEx Lift;

    // Global Variables:

    // Tunables:
    public static final double LP = 8, LI = 1, LD = 0.1, LF = 10; // Lift PIDF Values                                       | Already Tuned
    public static final double panMin = 0.210, panMax = 0.590, tiltMin = 0.350, tiltMax = 0.800; // Camera Servo Limits     | Already Tuned
                               // Left Max     // Right Max    // Down Max      // Up Max
    public static final double panHome = 0.500, tiltHome = 0.475; // Camera "Home" Position                                 | Already Tuned
    public static final double panScore = 0.500, tiltScore = 0.800; // Camera "Score" Position                              | Already Tuned
    public static final int colorThresholdDefault = 90; // ToDo:                                                            | Change For Each Environment?
    public static final int alphaThresholdDefault = 605; // ToDo:                                                           | Change For Each Environment?
    public static final boolean robotCanMove = true; // ToDo: Can the robot move on the table?
    public static final boolean robotQuietMode = false; // If we want less motor wining. ToDo: Maybe for enclosed environments?
    boolean liftActive = !robotQuietMode; // Sub-Variable for Quiet Mode, Don't Change

    // Others:
    boolean WiggleDir = false; // True for In, False for Out
    public int colorThreshold, alphaThreshold; // Used for active tuning

    // Tuning Mode Vars:
    boolean tuningFirstTime = true;
    boolean blockDetected;
    String blockColor;
    int selectedAction = 1;
    String selector;
    boolean editing = false;

    // Mode Latches:
    public boolean IdleModeActive = true;
    public boolean ActiveModeActive = false;
    public boolean TuningModeActive = false;

    // Global Enums:
    public enum IntakeAction {COLLECT, REJECT, SCORE, STOP}

    //endregion

    //region MainInit

    @Override
    public void mainInit() { // TODO: DO NOT ADD MultipleTelemetry(FtcDashboard), the telemetry is setup for the DRIVER HUB, NOT FTC DASHBOARD!
        DisplayUtils.init.initTelemetry(telemetry);
        DisplayUtils.init.initGamepad1(gamepad1);

        DisplayUtils.init.setTelemetryTransmissionRate(20);

        DisplayUtils.telemetry.log.addLine("Setup ~1% Complete: Int Hardware Map...");

        // [SETUP] Hardware Map
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        SparkFunOTOS sparkFun = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        colorSensor = hardwareMap.colorSensor.get("ColorSensor");
        IntakeServo1 = hardwareMap.get(CRServo.class, "IntakeServo1");
        IntakeServo2 = hardwareMap.get(CRServo.class, "IntakeServo2");
        CamServoPan = hardwareMap.get(Servo.class, "CamServoPan");
        CamServoTilt = hardwareMap.get(Servo.class, "CamServoTilt");
        Lift = hardwareMap.get(DcMotorEx.class, "lift");

        DisplayUtils.telemetry.log.addLine("Setup ~16% Complete: Motor Config... (1/2)");

        // [SETUP] Motor Config.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotor.Direction.REVERSE);

        Lift.setVelocityPIDFCoefficients(LP, LI, LD, LF);

        DisplayUtils.telemetry.log.addLine("Setup ~50% Complete: Motor Config... (2/2)");

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        DisplayUtils.telemetry.log.addLine("Setup ~66% Complete: Init Robot.java & DisplayUtils.java Link...");

        // [SETUP] Robot.java Link
        robot = Robot
                .getInstance(frontLeft, frontRight, backLeft, backRight)
                .setImu(sparkFun);

        DisplayUtils.telemetry.log.addLine("Setup ~83% Complete: IMU Config...");

        // [SETUP] IMU Config.
        configureOtos();

        DisplayUtils.telemetry.log.addLine("Setup ~99% Complete: Setting Robot Defaults...");

        // [SETUP] Defaults
        robot.stopMotors();
        cameraGotoPos(panHome, tiltHome);
        liftGotoPos(30);

        colorThreshold = colorThresholdDefault;
        alphaThreshold = alphaThresholdDefault;

        DisplayUtils.telemetry.log.addLine("RobotDefaults: colorThreshold:" + colorThresholdDefault + ", alphaThreshold:" + alphaThresholdDefault + ", canRobotMove?:" + robotCanMove);

        DisplayUtils.telemetry.log.addLine("Setup 100% Complete, Status: Waiting for start...");

        DisplayUtils.gamepad.led.softPulseLED(DisplayUtils.GamepadTarget.GAMEPAD1, 1, 1, 0, 0, 0, 0, 1000, 100, DisplayUtils.BlinkType.ODD_LOW);
        DisplayUtils.gamepad.rumble.advRumble(DisplayUtils.GamepadTarget.GAMEPAD1, 0.05, 0.05, 500);
    }

    //endregion

    //region Main

    @Override
    public void mainLoop() {

        if (IdleModeActive) {
            DisplayUtils.gamepad.led.setLED(DisplayUtils.GamepadTarget.GAMEPAD1, 1, 1, 0, -1);
            sleepForRand(500, 2000);

            int idleChance = ThreadLocalRandom.current().nextInt(1, 101);

            if (robotQuietMode) {
                if (idleChance <= 80) {
                    // 80% chance - Move Camera
                    DisplayUtils.telemetry.log.addLine("Current Idle Movement: Move Camera");
                    cameraGotoPos(ThreadLocalRandom.current().nextDouble(panMin, panMax - 0.05), ThreadLocalRandom.current().nextDouble(tiltMin, tiltMax - 0.2));
                } else {
                    // 20% chance - Wiggle Intake
                    DisplayUtils.telemetry.log.addLine("Current Idle Movement: Wiggle Intake");
                    intakeIdleWiggle();
                }
            } else {
            if (idleChance <= 70) {
                // 70% chance - Move Camera
                DisplayUtils.telemetry.log.addLine("Current Idle Movement: Move Camera");
                cameraGotoPos(ThreadLocalRandom.current().nextDouble(panMin, panMax - 0.05), ThreadLocalRandom.current().nextDouble(tiltMin, tiltMax - 0.2));
            } else if (idleChance <= 90) {
                // 20% chance - Wiggle Intake
                DisplayUtils.telemetry.log.addLine("Current Idle Movement: Wiggle Intake");
                intakeIdleWiggle();
            } else {
                // 10% chance - Move Lift
                DisplayUtils.telemetry.log.addLine("Current Idle Movement: Move Lift");
                liftGotoPos(ThreadLocalRandom.current().nextInt(10, 301));
            }
            }

            if (gamepad1.cross) {
                ActiveModeActive = true;
                IdleModeActive = false;
            }
            if (gamepad1.triangle) {
                TuningModeActive = true;
                IdleModeActive = false;
            }
        }


        if (ActiveModeActive) {
            DisplayUtils.telemetry.log.addLine("Status: Running Active Mode...");
            DisplayUtils.gamepad.led.softPulseLED(DisplayUtils.GamepadTarget.GAMEPAD1, 1, 0, 0, 0, 0, 1, 2000, 100, DisplayUtils.BlinkType.EVEN);
            DisplayUtils.gamepad.rumble.advRumble(DisplayUtils.GamepadTarget.GAMEPAD1, 0.05, 0.05, 500);

            robot.getImu().calibrateImu();

            robot.getImu().resetTracking();

            SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
            robot.getImu().setPosition(currentPosition);

            liftActive = true;
            liftGotoPos(70);
            cameraGotoPos(panHome, tiltMin);

            boolean correctBlock = false;
            while (!correctBlock) {
                boolean block = false;

                intakeMove(IntakeAction.COLLECT);
                while (!block && !gamepad1.triangle) {
                    block = colorSensor.alpha() > alphaThreshold;
                }
                intakeMove(IntakeAction.STOP);

                if (block && colorSensor.red() > colorSensor.blue() + colorThreshold) { // If Red Block, Collect
                    correctBlock = true;
                    DisplayUtils.telemetry.log.addLine("ActiveMode: Detected block: RED -> Scoring...");
                    sleep(1000);
                } else if (block && colorSensor.blue() > colorSensor.red() + colorThreshold) { // If Blue Block, Reject
                    intakeMove(IntakeAction.REJECT);
                    DisplayUtils.telemetry.log.addLine("ActiveMode: Detected block: BLUE -> Rejecting...");
                    sleep(500);

                    // NEW: *Shakes Head* LoL
                    cameraGotoPos(panMin, tiltHome);
                    sleep(500);
                    cameraGotoPos(panMax, tiltHome);
                    sleep(500);
                    cameraGotoPos(panMin, tiltHome);
                    sleep(500);
                    cameraGotoPos(panMax, tiltHome);

                    sleep(1000);
                    cameraGotoPos(panHome, tiltMin);
                } else if (block) { // If Unidentifiable, Reject
                    intakeMove(IntakeAction.REJECT);
                    DisplayUtils.telemetry.log.addLine("<ERROR> Block color could not be identified! Rejecting...");
                    sleep(1000);
                }
            }

            liftGotoPos(120);
            sleep(500);

            cameraGotoPos(panHome, tiltHome);
            if (robotCanMove) {robot.goTo(0.3, "0", "12", "0");}

            sleep(500);

            cameraGotoPos(panScore, tiltScore);
            liftGotoPos(2850);

            sleep(500);
            intakeMove(IntakeAction.SCORE);

            sleep(1000);
            intakeMove(IntakeAction.STOP);
            liftGotoPos(30);

            sleep(500);
            cameraGotoPos(panHome, tiltHome);
            if (robotCanMove) {robot.goTo(0.3, "0", "0", "0");}
            sleep(500);

            gamepad1.rumble(0.1, 0.1, 300);
            liftActive = !robotQuietMode;
            liftGotoPos(0);

            ActiveModeActive = false;
            IdleModeActive = true;
        }


        if (TuningModeActive) {
            if (tuningFirstTime) {
                DisplayUtils.telemetry.log.addLine("Status: Running Tuning Mode...");
                DisplayUtils.gamepad.led.softPulseLED(DisplayUtils.GamepadTarget.GAMEPAD1, 0, 1, 0, 0, 0, 1, 2000, 100, DisplayUtils.BlinkType.EVEN);
                DisplayUtils.gamepad.rumble.advRumble(DisplayUtils.GamepadTarget.GAMEPAD1, 0.05, 0.05, 500);
                liftGotoPos(200);
                cameraGotoPos(panScore, tiltScore);

                telemetry.setAutoClear(false);
                tuningFirstTime = false;
            }

            DisplayUtils.telemetry.log.clearLog(false);
            telemetry.addLine("TUNING MODE\n");

            if (selectedAction == 1 && editing) {
                selector = ">>";
            } else if (selectedAction == 1) {
                selector = "> ";
            } else {
                selector = "  ";
            }
            telemetry.addLine(selector + "Color Threshold : " + colorThreshold + "(Default: " + colorThresholdDefault + ")");

            if (selectedAction == 2 && editing) {
                selector = ">>";
            } else if (selectedAction == 2) {
                selector = "> ";
            } else {
                selector = "  ";
            }
            telemetry.addLine(selector + "Alpha Threshold : " + alphaThreshold + "(Default: " + alphaThresholdDefault + ")");

            if (selectedAction == 3) {
                selector = "> ";
            } else {
                selector = "  ";
            }
            telemetry.addLine(selector + "EXIT TUNING MODE\n\n");

            telemetry.addLine("OUTPUT:\n");

            blockDetected = colorSensor.alpha() > alphaThreshold;
            telemetry.addLine("Block Detected? : " + blockDetected);

            if (colorSensor.red() > colorSensor.blue() + colorThreshold) {
                blockColor = "RED";
            } else if (colorSensor.blue() > colorSensor.red() + colorThreshold) {
                blockColor = "BLUE";
            } else {
                blockColor = "UNKNOWN";
            }
            telemetry.addLine("Block Color : " + blockColor);

            if (!editing) {
                if (gamepad1.dpad_up) {
                    sleep(100);
                    if (selectedAction != 1) {
                        selectedAction -= 1;
                    }
                }
                if (gamepad1.dpad_down) {
                    sleep(100);
                    if (selectedAction != 3) {
                        selectedAction += 1;
                    }
                }
                if (gamepad1.dpad_right) {
                    sleep(100);
                    if (selectedAction != 3) {
                        editing = true;
                    } else {
                        selectedAction = 1;
                        tuningFirstTime = true;
                        TuningModeActive = false;
                        IdleModeActive = true;
                        editing = false;
                    }
                }
            } else {
                if (gamepad1.dpad_up) {
                    sleep(100);
                    if (selectedAction == 1) {
                        colorThreshold += 5;
                    } else {
                        alphaThreshold += 5;
                    }
                }
                if (gamepad1.dpad_down) {
                    sleep(100);
                    if (selectedAction == 1) {
                        colorThreshold -= 5;
                    } else {
                        alphaThreshold -= 5;
                    }
                }
                if (gamepad1.dpad_left) {
                    sleep(100);
                    editing = false;
                }
                sleep(80);
            }
        }
        // ToDo: Note for Part 2: setTheToggle(</NaN/>), THEN: goToPos(30);
    }

    //endregion

    //region Exe Functions

    // Less Main Stuffs:
    public void liftGotoPos(int LiftPos) {
        if (liftActive) {
            Lift.setMotorEnable();
            if (LiftPos > 2900 || LiftPos < -10) {
                throw new IllegalStateException("Lift Pos is OUT OF BOUNDS!");
            }

            Lift.setTargetPosition(LiftPos);
            Lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Lift.setVelocity(500); // This is the MAX velocity the internal PID will aim for

            //noinspection StatementWithEmptyBody
            while (Lift.isBusy()) ; // THIS IS REQUIRED!
        } else {
            Lift.setMotorDisable();
            DisplayUtils.telemetry.log.addLine("<ERROR> [SILENT] Lift was called, but lift is not active!");
        }
    }


    public void cameraGotoPos(double pan, double tilt) {
        if (!(pan >= panMin && pan <= panMax && tilt >= tiltMin && tilt <= tiltMax)) {
            throw new IllegalStateException("Camera Pos is OUT OF BOUNDS!");
        }
        CamServoPan.setPosition(pan);
        CamServoTilt.setPosition(tilt);
    }

    public void intakeMove(IntakeAction action) {
        switch (action) {
            case COLLECT:
                IntakeServo1.setPower(0.1);
                IntakeServo2.setPower(-0.1);
                break;
            case REJECT:
                IntakeServo1.setPower(-0.5);
                IntakeServo2.setPower(0.5);
                break;
            case SCORE:
                IntakeServo1.setPower(-1);
                IntakeServo2.setPower(1);
                break;
            case STOP:
                IntakeServo1.setPower(0);
                IntakeServo2.setPower(0);
                break;
            default:
                throw new IllegalStateException(action + " is an INVALID intake action!");
        }
    }

    public void intakeIdleWiggle() {
        if (WiggleDir) {
            intakeMove(IntakeAction.COLLECT);
            sleep(1000);
            intakeMove(IntakeAction.STOP);
            WiggleDir = false;
        } else {
            intakeMove(IntakeAction.REJECT);
            sleep(500);
            intakeMove(IntakeAction.STOP);
            WiggleDir = true;
        }
    }

    public void sleepForRand(long MinMillis, long MaxMillis) {
        if (MinMillis >= MaxMillis || MinMillis < 0) {
            throw new IllegalStateException("MIN value is greater than or equal to MAX value! OR MIN is < 0!");
        }
        if (MaxMillis >= Long.MAX_VALUE - 3) {
            throw new IllegalStateException("Why wait so long!?");
        }
        long randomTime = ThreadLocalRandom.current().nextLong(MinMillis, MaxMillis + 1);
        sleep(randomTime);
    }

    //endregion

    //region OpModeShutdown

    @Override
    protected void onOpModeStop() {
        DisplayUtils.telemetry.log.addLine("Status: Shutting Down...");

        intakeMove(IntakeAction.STOP);
        cameraGotoPos(panScore, tiltScore);
        liftGotoPos(0);
    }

    //endregion

    //region SparkFunOTOS Config
    @SuppressLint("DefaultLocale")
    private void configureOtos() {
        // SparkFun.setLinearUnit(DistanceUnit.METER);
        robot.getImu().setLinearUnit(DistanceUnit.INCH);
        // SparkFun.setAngularUnit(AngleUnit.RADIANS);
        robot.getImu().setAngularUnit(AngleUnit.DEGREES);

        robot.getImu().setLinearScalar(1.0);
        robot.getImu().setAngularScalar(1.0);

        robot.getImu().calibrateImu();

        robot.getImu().resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        robot.getImu().setPosition(currentPosition);
    }
    //endregion
}
