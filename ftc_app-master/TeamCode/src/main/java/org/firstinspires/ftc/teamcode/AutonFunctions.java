package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import static org.firstinspires.ftc.teamcode.GoldAlignDetector.Pos.*;
import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;



public class AutonFunctions {
    // System Variables
    private static ElapsedTime runtime = new ElapsedTime();
    private static HardwareMap hwMap = null;
    private static Telemetry telemetry = null;

    // Drive variables
    private static DcMotor leftDrive = null;
    private static DcMotor rightDrive = null;
    static double leftDrivePower = 0;
    static double rightDrivePower = 0;
    static double leftTotalDist = 0;
    static double rightTotalDist = 0;
    static final double DRIVE_ENC_ERR_RANGE = 50;
    static final double DRIVE_COUNTS_PER_MOTOR_REV = 1120;  // Per Rev Documentation, 1120 counts per output shaft revolution
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_INCHES = 3.5;
    static final double DRIVE_COUNTS_PER_INCH = (DRIVE_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Elev Variables
    private static DcMotor leftElev = null;
    private static DcMotor rightElev = null;
    static double leftElevPower = 0;
    static double rightElevPower = 0;
    static final double ELEV_ENC_ERR_RANGE = 25;

    // Limit switch Variabless
    private static DigitalChannel lowerLimit = null;
    private static DigitalChannel upperLimit = null;

    // Index
    public static int index = 0;

    // Timer reset flag
    private static boolean isNew = true;

    // Vision variables
    public static GoldAlignDetector detector = new GoldAlignDetector();
    public static double filterThreshold = 90;

    // Gold position
    public static GoldAlignDetector.Pos goldPos;


    public static void initializeAuton(HardwareMap ahwMap, Telemetry atelemetry){
        telemetry = atelemetry;
        hwMap = ahwMap;

        telemetry.addData("Status", "Initializing...");
        // Set drive parameters
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);  // Opposite of teleop becuase of joystick values
        rightDrive.setDirection(DcMotor.Direction.FORWARD);  // Opposite of teleop becuase of joystick values
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set elevator parameters
        leftElev = hwMap.get(DcMotor.class, "left_elev");
        rightElev = hwMap.get(DcMotor.class, "right_elev");
        leftElev.setDirection(DcMotor.Direction.REVERSE);
        rightElev.setDirection(DcMotor.Direction.FORWARD);
        rightElev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Encoder on Right Elev only
        rightElev.setMode(DcMotor.RunMode.RUN_TO_POSITION);         // Encoder on Right Elev only
        leftElev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightElev.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set limit switch parameters
        lowerLimit = hwMap.digitalChannel.get("lower_limit");
        upperLimit = hwMap.digitalChannel.get("upper_limit");
        lowerLimit.setMode(DigitalChannel.Mode.INPUT);
        upperLimit.setMode(DigitalChannel.Mode.INPUT);

        index = 0;
        leftTotalDist = 0;
        rightTotalDist = 0;
        isNew = true;

        telemetry.addData("Status", "Initialized");
    }

    public static void drive(double leftDistance, double rightDistance, double motorPwr) {
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int leftDriveEncCount;
        int rightDriveEncCount;
        boolean leftDriveFinished = FALSE;
        boolean rightDriveFinished = FALSE;

        if (isNew) {
            leftTotalDist += leftDistance;
            rightTotalDist += rightDistance;
            isNew = false;
        }
        leftDriveEncCount = (int) Math.round(leftTotalDist * DRIVE_COUNTS_PER_INCH);
        rightDriveEncCount = (int) Math.round(rightTotalDist * DRIVE_COUNTS_PER_INCH);
        telemetry.addData("left enc", "(%d)", leftDriveEncCount);
        telemetry.addData("right enc", "(%d)", rightDriveEncCount);
        if ((leftDrive.getCurrentPosition() >= leftDriveEncCount - DRIVE_ENC_ERR_RANGE) &&
                (leftDrive.getCurrentPosition() <= leftDriveEncCount + DRIVE_ENC_ERR_RANGE)) {
            leftDrivePower = 0;
            leftDriveFinished = TRUE;
        }
        else{
            leftDrive.setTargetPosition(leftDriveEncCount);
            leftDrivePower = motorPwr;
        }

        if ((rightDrive.getCurrentPosition() >= rightDriveEncCount - DRIVE_ENC_ERR_RANGE) &&
                (rightDrive.getCurrentPosition() <= rightDriveEncCount + DRIVE_ENC_ERR_RANGE)) {
            rightDrivePower = 0;
            rightDriveFinished = TRUE;
        }
        else{
            rightDrive.setTargetPosition(rightDriveEncCount);
            rightDrivePower = motorPwr;
        }

        if (leftDriveFinished && rightDriveFinished) {
            isNew = true;
            index++;
        }
        leftDrive.setPower(leftDrivePower);
        rightDrive.setPower(rightDrivePower);
    }

    public static void elev(int elevTarget){
        boolean lowerLimitPressed = !lowerLimit.getState();
        boolean upperLimitPressed = !upperLimit.getState();

//        if (lowerLimitPressed) {rightElev.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}
        telemetry.addData("Booleans", "(%b), (%b), (%b), (%b)", rightElev.getCurrentPosition() >= elevTarget - ELEV_ENC_ERR_RANGE,
                rightElev.getCurrentPosition() <= elevTarget + ELEV_ENC_ERR_RANGE,
                elevTarget > 300 && upperLimitPressed, elevTarget < 300 && lowerLimitPressed);

        if (((rightElev.getCurrentPosition() >= elevTarget - ELEV_ENC_ERR_RANGE)
                && (rightElev.getCurrentPosition() <= elevTarget + ELEV_ENC_ERR_RANGE))
                || (elevTarget > 300 && upperLimitPressed) || (elevTarget < 300 && lowerLimitPressed)) {
            leftElevPower = 0;
            rightElevPower = 0;
            index++;
        }
        else {
            rightElev.setTargetPosition(elevTarget);
            leftElevPower = 1;
            rightElevPower = 1;
        }

//        leftElev.setPower(leftElevPower);
        telemetry.addData("Elev Power", "(%.2f)", rightElev.getPower());
        rightElev.setPower(rightElevPower);
    }

    public static void setElevZeroPwr(DcMotor.ZeroPowerBehavior mode) {
        leftElev.setZeroPowerBehavior(mode);
    }

    public static void delay(double durationInMillseconds) {
        if (isNew) {
            runtime.reset();
            isNew = false;
        }
        if (runtime.milliseconds() >= durationInMillseconds) {
            isNew = true;
            index++;
        }
    }

    public static void allStop() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftElev.setPower(0);
        rightElev.setPower(0);
    }

    public static void sendTelemetry() {
        telemetry.addData("Drive", "left:  (%.2f), right:  (%.2f)", leftDrivePower, rightDrivePower);
        telemetry.addData("Drive Enc", "left:  (%d), right:  (%d)", leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Elev", "left:  (%.2f), right:  (%.2f)", leftElevPower, rightElevPower);
        telemetry.addData("Elev Enc", "left:  (%d)", rightElev.getCurrentPosition());
        telemetry.addData("State", index);
    }

    public static void initVision() {
        detector.init(hwMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 75; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
        index++;
    }

    public static void findGold() {
        if (detector.isFound()) {
            if (index == 11) {
                goldPos = LEFT;
            }
            alignGold();
        }
        else
            index++;
    }

    public static void alignGold() {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (detector.getAligned()) {
            allStop();
            detector.disable();
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftTotalDist = 0;
            rightTotalDist = 0;
            index = 12;       // Skip the rest of the checks for gold
        }
        else if (detector.goldFieldPos == RIGHT) {
            leftDrive.setPower(.3);
            rightDrive.setPower(0);
        }
        else if (detector.goldFieldPos == LEFT) {
            leftDrive.setPower(0);
            rightDrive.setPower(.3);
        }
    }

}
