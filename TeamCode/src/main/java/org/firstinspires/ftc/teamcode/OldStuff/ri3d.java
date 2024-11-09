package org.firstinspires.ftc.teamcode.OldStuff;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ri3d {
    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor leftBackDrive;
    DcMotor rightBackDrive;
    private double countsPerInch;
    private ElapsedTime runtime = new ElapsedTime();
    private OpMode theOpMode;
    private DistanceSensor leftSensorRange;
    private DistanceSensor rightSensorRange;
    ModernRoboticsI2cRangeSensor rangeSensor;
    private ModernRoboticsI2cGyro gyroSensor;
    private DistanceSensor distanceSensor1;
    NormalizedColorSensor colorSensor;
    static final double WHITE_THRESHOLD = 0.5;  // spans between 0.0 - 1.0 from dark to light
    static final double APPROACH_SPEED = 0.25;

    //Teleop Variables
    Orientation angles;
    BNO055IMU imuCH;
    double drvTrnSpd = .75;
    float IMUReading = 0;
    double startingHeadingInRadians = 0;
    boolean upFlag = false;
    boolean upPersistent = false;
    boolean downFlag = false;
    boolean downPersistent = false;
    int count = 0;
    double[] angleTest = new double[10];
    double average;
    double correct;
    double STARTING_HEADING = 0;
    boolean autoTurnEnabled = false;
    double ZeroPosition = Math.toRadians(180);
    double AbsoluteValue = 0;
    private Orientation lastAngles = new Orientation();
    private double currAngle = 0.0;
    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor
    double  power   = 0;
    boolean rampUp  = true;


    public ri3d(HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        theOpMode = opMode;
        theOpMode.telemetry.addData("Running to", "here");
        theOpMode.telemetry.update();
        leftDrive = hardwareMap.dcMotor.get("leftDrive");
        rightDrive = hardwareMap.dcMotor.get("rightDrive");
        leftBackDrive = hardwareMap.dcMotor.get("leftBackDrive");
        rightBackDrive = hardwareMap.dcMotor.get("rightBackDrive");


        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imuCH = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imuCH.initialize(parameters);
        if (!leftDrive.isBusy()) {
            leftDrive.setTargetPosition(leftDrive.getCurrentPosition());
            rightDrive.setTargetPosition(rightDrive.getCurrentPosition());
            leftBackDrive.setTargetPosition(leftBackDrive.getCurrentPosition());
            rightBackDrive.setTargetPosition(rightBackDrive.getCurrentPosition());
        }

    }

    public void encoderDrive2(double speed, double inches, double timeoutS) {

        // Target values for wheels (motors)
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Setting the target values to their new positions
        newLeftTarget = leftDrive.getCurrentPosition() + (int) (inches * countsPerInch);
        newRightTarget = rightDrive.getCurrentPosition() + (int) (inches * countsPerInch);
        newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (inches * countsPerInch);
        newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (inches * countsPerInch);

        // Gives the motor(s) their target destination
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        leftBackDrive.setTargetPosition(newLeftBackTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);

        double error = newLeftTarget - leftDrive.getCurrentPosition();


        // Sets the motors' mode to RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // resets timeout time
        runtime.reset();

        // Starts motion
        leftDrive.setPower(Math.abs(error));
        rightDrive.setPower(Math.abs(error));
        leftBackDrive.setPower(Math.abs(error));
        rightBackDrive.setPower(Math.abs(error));

        while (((LinearOpMode)theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftDrive.isBusy() && rightDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
            leftDrive.setPower(Math.abs(error));
            rightDrive.setPower(Math.abs(error));
            leftBackDrive.setPower(Math.abs(error));
            rightBackDrive.setPower(Math.abs(error));

            //as far as I can tell, this is purely for display
            theOpMode.telemetry.addData("Currently at", " at %7d :%7d, %7d, %7d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            theOpMode.telemetry.addData("Current Error", error);
            theOpMode.telemetry.update();

        }


        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void encoderDrive(double speed, double inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() + (int) (inches * countsPerInch);
        newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (inches * countsPerInch);
        newRightTarget = rightDrive.getCurrentPosition() + (int) (inches * countsPerInch);
        newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (inches * countsPerInch);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        leftBackDrive.setTargetPosition(newLeftBackTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));

        while (((LinearOpMode)theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftDrive.isBusy() && rightDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

            // Display it for the driver.
            theOpMode.telemetry.addData("Running to", " %7d :%7d, %7d, %7d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
            theOpMode.telemetry.addData("Currently at", " at %7d :%7d, %7d, %7d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            theOpMode.telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(-speed);
        rightDrive.setPower(-speed);
        leftBackDrive.setPower(-speed);
        rightBackDrive.setPower(-speed);
        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

    }
    public void encoderDriveStart(double speed, double inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() + (int) (inches * countsPerInch);
        newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int) (inches * countsPerInch);
        newRightTarget = rightDrive.getCurrentPosition() + (int) (inches * countsPerInch);
        newRightBackTarget = rightBackDrive.getCurrentPosition() + (int) (inches * countsPerInch);

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        leftBackDrive.setTargetPosition(newLeftBackTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));
    }

    public boolean encoderDriveCheck(double speed, double inches, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftDrive.isBusy() && rightDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

            // Display it for the driver.
//            theOpMode.telemetry.addData("Running to", " %7d :%7d, %7d, %7d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
            theOpMode.telemetry.addData("Currently at", " at %7d :%7d, %7d, %7d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            theOpMode.telemetry.update();
            return true;
        }
        else {
            encoderDriveEnd();
            return false;
        }

    }

    public void encoderDriveEnd() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void strafeEncoderDrive(double speed, double inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() - (int) (inches * countsPerInch);
        newLeftBackTarget = (leftBackDrive.getCurrentPosition() + (int) (inches * countsPerInch));
        newRightTarget = (rightDrive.getCurrentPosition() + (int) (inches * countsPerInch));
        newRightBackTarget = (rightBackDrive.getCurrentPosition() - (int) (inches * countsPerInch));

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        leftBackDrive.setTargetPosition(newLeftBackTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed) * .7);
        rightDrive.setPower(Math.abs(speed) * .7);
        leftBackDrive.setPower(Math.abs(-speed) * 1.2);
        rightBackDrive.setPower(Math.abs(-speed) * 1.2);

        while (((LinearOpMode) theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftDrive.isBusy() && rightDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {

            // Display it for the driver.
            theOpMode.telemetry.addData("Running to", " %7d :%7d, %7d, %7d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
            theOpMode.telemetry.addData("Currently at", " at %7d :%7d, %7d, %7d",
                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            theOpMode.telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void strafeEncoderDriveStart(double speed, double inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() - (int) (inches * countsPerInch);
        newLeftBackTarget = (leftBackDrive.getCurrentPosition() + (int) (inches * countsPerInch));
        newRightTarget = (rightDrive.getCurrentPosition() + (int) (inches * countsPerInch));
        newRightBackTarget = (rightBackDrive.getCurrentPosition() - (int) (inches * countsPerInch));

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        leftBackDrive.setTargetPosition(newLeftBackTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));
        leftBackDrive.setPower(Math.abs(-speed));
        rightBackDrive.setPower(Math.abs(-speed));
    }

    public boolean strafeEncoderDriveCheck(double speed, double inches, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftDrive.isBusy() && rightDrive.isBusy() && leftBackDrive.isBusy() && rightBackDrive.isBusy())) {
            return true;
            // Display it for the driver.
//            theOpMode.telemetry.addData("Running to", " %7d :%7d, %7d, %7d", newLeftTarget, newRightTarget, newLeftBackTarget, newRightBackTarget);
//            theOpMode.telemetry.addData("Currently at", " at %7d :%7d, %7d, %7d",
//                    leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
//            theOpMode.telemetry.update();
        } else {
            strafeEncoderDriveEnd();
            return false;
        }
    }

    public void strafeEncoderDriveEnd() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeEncoderDriveTest(double speed, double inches, double timeoutS) {
        strafeEncoderDriveStart(speed, inches, timeoutS);
        strafeEncoderDriveCheck(speed, inches, timeoutS);
    }


    public void driveToDistanceSensor(double APPROACH_SPEED, double inches, double timeoutS) {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double sensorReading = Double.MAX_VALUE;
        runtime.reset();
        leftDrive.setPower(APPROACH_SPEED);
        rightDrive.setPower(APPROACH_SPEED);
        leftBackDrive.setPower(APPROACH_SPEED);
        rightBackDrive.setPower(APPROACH_SPEED);
        while (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS) && sensorReading > inches) {
            sensorReading = distanceSensor1.getDistance(DistanceUnit.INCH);
            theOpMode.telemetry.addData("Seconds vs timeout", " %.2f < %.4f", runtime.seconds(), timeoutS);
            theOpMode.telemetry.addData("Running to", " %.2f", inches);
            theOpMode.telemetry.addData("Currently at", " at %.2f", sensorReading);
            theOpMode.telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void driveToRangeSensor(double APPROACH_SPEED, double inches, double timeoutS) {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double sensorReading = Double.MAX_VALUE;
        runtime.reset();
        leftDrive.setPower(APPROACH_SPEED);
        rightDrive.setPower(APPROACH_SPEED);
        leftBackDrive.setPower(APPROACH_SPEED);
        rightBackDrive.setPower(APPROACH_SPEED);
        while (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS) && sensorReading > inches) {
            sensorReading = rangeSensor.getDistance(DistanceUnit.INCH);
            theOpMode.telemetry.addData("Seconds vs timeout", " %.2f < %.4f", runtime.seconds(), timeoutS);
            theOpMode.telemetry.addData("Running to", " %.2f", inches);
            theOpMode.telemetry.addData("Currently at", " at %.2f", sensorReading);
            theOpMode.telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void driveToDistanceSensorBackwards(double APPROACH_SPEED, double inches, double timeoutS) {
        double sensorReading = distanceSensor1.getDistance(DistanceUnit.INCH);
        runtime.reset();
        leftDrive.setPower(APPROACH_SPEED);
        rightDrive.setPower(APPROACH_SPEED);
        leftBackDrive.setPower(APPROACH_SPEED);
        rightBackDrive.setPower(APPROACH_SPEED);
        while (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS) && sensorReading < inches) {

            sensorReading = distanceSensor1.getDistance(DistanceUnit.INCH);
            theOpMode.telemetry.addData("Seconds vs timeout", " %.2f < %.4f", runtime.seconds(), timeoutS);
            theOpMode.telemetry.addData("Running to", " %.2f", inches);
            theOpMode.telemetry.addData("Currently at", " at %.2f", sensorReading);
            theOpMode.telemetry.update();

        }

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void strafeToDistanceSensorBackwards(double APPROACH_SPEED, double inches, double timeoutS) {
        double sensorReading = 0.2;
        runtime.reset();
        leftDrive.setPower(-APPROACH_SPEED * .9);
        rightDrive.setPower(APPROACH_SPEED * .91);
        leftBackDrive.setPower(APPROACH_SPEED);
        rightBackDrive.setPower(-APPROACH_SPEED);
        while (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS) && sensorReading < inches) {
            sensorReading = distanceSensor1.getDistance(DistanceUnit.INCH);
            theOpMode.telemetry.addData("Seconds vs timeout", " %.2f < %.4f", runtime.seconds(), timeoutS);
            theOpMode.telemetry.addData("Running to", " %.2f", inches);
            theOpMode.telemetry.addData("Currently at", " at %.2f", sensorReading);
            theOpMode.telemetry.update();

        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void driveToDistanceSensorStart(double APPROACH_SPEED, double inches, double timeoutS) {
        double sensorReading = 5001;
        runtime.reset();
        leftDrive.setPower(APPROACH_SPEED);
        rightDrive.setPower(APPROACH_SPEED);
        leftBackDrive.setPower(APPROACH_SPEED);
        rightBackDrive.setPower(APPROACH_SPEED);

    }
    public void driveToPlace(double speed) {

    }

    public boolean driveToDistanceSensorCheck(double APPROACH_SPEED, double inches, boolean isRed, double timeoutS) {
        double sensorReading = 5001;
        sensorReading = distanceSensor1.getDistance(DistanceUnit.INCH);
        if (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS) && sensorReading > inches) {

            //  theOpMode.telemetry.addData("Seconds vs timeout", " %.2f < %.4f", runtime.seconds(), timeoutS);
            //  theOpMode.telemetry.addData("Running to", " %.2f", inches);
            //  theOpMode.telemetry.addData("Currently at", " at %.2f", sensorReading);
            // theOpMode.telemetry.update();
            return true;
        } else {
            driveToDistanceSensorEnd();
            return false;
        }
    }

    public void driveToDistanceSensorEnd() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void strafeToDistanceSensor(double APPROACH_SPEED, double inches, double timeoutS) {
        double sensorReading = Double.MAX_VALUE;
        runtime.reset();
        leftDrive.setPower(-APPROACH_SPEED * .9);
        rightDrive.setPower(APPROACH_SPEED * .91);
        leftBackDrive.setPower(APPROACH_SPEED);
        rightBackDrive.setPower(-APPROACH_SPEED);
        while (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS) && sensorReading > inches) {
            sensorReading = distanceSensor1.getDistance(DistanceUnit.INCH);
            theOpMode.telemetry.addData("Seconds vs timeout", " %.2f < %.4f", runtime.seconds(), timeoutS);
            theOpMode.telemetry.addData("Running to", " %.2f", inches);
            theOpMode.telemetry.addData("Currently at", " at %.2f", sensorReading);
            theOpMode.telemetry.update();

        }
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void strafeToDistanceSensorStart(double APPROACH_SPEED, double inches, double timeoutS) {
        double sensorReading = 5001;


        runtime.reset();
        leftDrive.setPower(-APPROACH_SPEED);
        rightDrive.setPower(APPROACH_SPEED);
        leftBackDrive.setPower(APPROACH_SPEED);
        rightBackDrive.setPower(-APPROACH_SPEED);
    }

    public boolean strafeToDistanceSensorCheck(double APPROACH_SPEED, double inches, double timeoutS) {
        //theOpMode.telemetry.addData("Currently at", " at %.2f", sensorReading);
        //            theOpMode.telemetry.update();

        double sensorReading = 5001;
        if (((LinearOpMode) theOpMode).opModeIsActive() && (runtime.seconds() < timeoutS) && sensorReading > inches) {
            sensorReading = distanceSensor1.getDistance(DistanceUnit.INCH);
//            theOpMode.telemetry.addData("Seconds vs timeout", " %.2f < %.4f", runtime.seconds(), timeoutS);
//            theOpMode.telemetry.addData("Running to", " %.2f", inches);
//            theOpMode.telemetry.addData("Currently at", " at %.2f", sensorReading);
//            theOpMode.telemetry.update();
            return true;
        }
        else {
            strafeToDistanceSensorEnd();
            return false;
        }
    }



    public void strafeToDistanceSensorEnd() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    public void diagonalDriveRight(double speed, double inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() - (int) (inches * countsPerInch);
        newRightBackTarget = (rightBackDrive.getCurrentPosition() - (int) (inches * countsPerInch));

        leftDrive.setTargetPosition(newLeftTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed));
        rightBackDrive.setPower(Math.abs(speed));

        while (((LinearOpMode) theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftDrive.isBusy() && rightBackDrive.isBusy())) {

            // Display it for the driver.
            theOpMode.telemetry.addData("Running to", " %7d :%7d,", newLeftTarget, newRightBackTarget);
            theOpMode.telemetry.addData("Currently at", " at %7d :%7d,",
                    leftDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            theOpMode.telemetry.update();
        }

        // Stop all motion;
        leftDrive.setPower(0);
        rightBackDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void diagonalDriveLeft(double speed, double inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftBackDrive.getCurrentPosition() - (int) (inches * countsPerInch);
        newRightBackTarget = (rightDrive.getCurrentPosition() - (int) (inches * countsPerInch));

        leftBackDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        leftBackDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));

        while (((LinearOpMode) theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftBackDrive.isBusy() && rightDrive.isBusy())) {

            // Display it for the driver.
            theOpMode.telemetry.addData("Running to", " %7d :%7d,", newLeftTarget, newRightBackTarget);
            theOpMode.telemetry.addData("Currently at", " at %7d :%7d,",
                    leftBackDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
            theOpMode.telemetry.update();
        }

        // Stop all motion;
        leftBackDrive.setPower(0);
        rightDrive.setPower(0);

        // Turn off RUN_TO_POSITION
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void pidDrive(double speed, double inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;
        while (((LinearOpMode) theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (leftBackDrive.isBusy() && rightDrive.isBusy())) {

        }
    }



    public void driveToColorSensor(double APPROACH_SPEED, boolean isRed, double timeoutS) {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        double sensorReading = 0.0;


        // run until the white line is seen OR the driver presses STOP;
        leftDrive.setPower(APPROACH_SPEED);
        rightDrive.setPower(APPROACH_SPEED);
        leftBackDrive.setPower(APPROACH_SPEED);
        rightBackDrive.setPower(APPROACH_SPEED);
        colorSensor.setGain(70f);

        //   while (theOpMode.opModeIsActive() && (getBrightness() < WHITE_THRESHOLD)) {
        while (((LinearOpMode) theOpMode).opModeIsActive() && sensorReading < 0.38 && runtime.seconds() < timeoutS) {

            if (isRed) {
                sensorReading = colorSensor.getNormalizedColors().red;
            } else {
                sensorReading = colorSensor.getNormalizedColors().blue;
            }
            // Send telemetry message to signify robot waiting;

            // Display the light level while we are waiting to start
            getBrightness();
        }


        // Stop all motors
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    double getBrightness() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        theOpMode.telemetry.addData("Light Level (0 to 1)", "%4.2f", colors.alpha);
        theOpMode.telemetry.addData("Blue Level (0 to 1)", "%4.2f", colors.blue);
        theOpMode.telemetry.addData("Red Level (0 to 1)", "%4.2f", colors.red);
        theOpMode.telemetry.update();

        return colors.alpha;

    }
    public void resetAngle() {
        lastAngles = imuCH.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {
        // Get current orientation
        Orientation orientation = imuCH.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;
        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }
        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        theOpMode.telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void turn(double degrees) {
        resetAngle();
        double error = degrees;
        while (((LinearOpMode) theOpMode).opModeIsActive() && Math.abs(error) > 2) {
            double motorSpeed = ((error < 0 ? -0.3 : 0.3));
            leftDrive.setPower(motorSpeed);
            rightDrive.setPower(motorSpeed);
            leftBackDrive.setPower(motorSpeed);
            rightBackDrive.setPower(motorSpeed);
            error = degrees - getAngle();
            theOpMode.telemetry.addData("error", error);
            theOpMode.telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void turnTo(double degrees) {
        Orientation orientation = imuCH.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;
        if (error > 180) {
            error -= 360;
        } else if (error > -180) {
            error += 360;
        }
        turn(error);
    }
    public void turnToTable(double degrees) {
        Orientation orientation = imuCH.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;
        if (error > 180) {
            error -= 360;
        } else if (error > -180) {
            error += 360;
        }
        turn(error);
    }
    public double getAbsoluteAngle() {
        return imuCH.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void turnPID(double degrees, double timeoutS) {
        turnToPID(degrees + getAbsoluteAngle(), timeoutS);
    }

    void turnToPID(double targetAngle, double timeoutS) {
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TurnPIDController pid = new TurnPIDController(targetAngle, 0.04, 0.00000008, 0.0001);
        //theOpMode.telemetry.setMsTransmissionInterval(50);
        double degreeCount = 0;
        runtime.reset();
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while ((Math.abs(targetAngle - getAbsoluteAngle()) > 1 || pid.getLastSlope() > 1.25 || degreeCount < 4) && runtime.seconds() < timeoutS) {
            if (Math.abs(targetAngle - getAbsoluteAngle()) < 1){
                degreeCount +=1;
            }
            double motorSpeed = pid.update(getAbsoluteAngle());
            leftDrive.setPower(motorSpeed);
            rightDrive.setPower(-motorSpeed);
            leftBackDrive.setPower(motorSpeed * 1.1);
            rightBackDrive.setPower(-motorSpeed * 1.1);
            theOpMode.telemetry.addData("degreeCount", degreeCount);
            theOpMode.telemetry.addData("Current Angle", getAbsoluteAngle());
            theOpMode.telemetry.addData("Target Angle", targetAngle);
            theOpMode.telemetry.addData("Slope", pid.getLastSlope());
            theOpMode.telemetry.addData("Power", motorSpeed);
            theOpMode.telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    void turnToPIDStart(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.05, 0.00000019, 0.00001);
        //theOpMode.telemetry.setMsTransmissionInterval(50);
        double degreeCount = 0;
    }

    public boolean turnToPIDCheck(double targetAngle) {
        double degreeCount = 0;
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.05, 0.00000019, 0.00001);

        if (Math.abs(targetAngle - getAbsoluteAngle()) > 1 || pid.getLastSlope() > 1.25 || degreeCount < 4) {
            if (Math.abs(targetAngle - getAbsoluteAngle()) < 1) {
                degreeCount += 1;
            }
            double motorSpeed = pid.update(getAbsoluteAngle());
            leftDrive.setPower(-motorSpeed);
            rightDrive.setPower(motorSpeed);
            leftBackDrive.setPower(-motorSpeed);
            rightBackDrive.setPower(motorSpeed);
            theOpMode.telemetry.addData("degreeCount", degreeCount);
            theOpMode.telemetry.addData("Current Angle", getAbsoluteAngle());
            theOpMode.telemetry.addData("Target Angle", targetAngle);
            theOpMode.telemetry.addData("Slope", pid.getLastSlope());
            theOpMode.telemetry.addData("Power", motorSpeed);
            theOpMode.telemetry.update();
            return true;
        }
        {
            turnToPIDEnd();
            return false;
        }
    }
    void turnToPIDEnd() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }



    public void strafeWithPID(double speed, double inches, double targetAngle, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() - (int) (inches * countsPerInch);
        newLeftBackTarget = (leftBackDrive.getCurrentPosition() + (int) (inches * countsPerInch));
        newRightTarget = (rightDrive.getCurrentPosition() + (int) (inches * countsPerInch));
        newRightBackTarget = (rightBackDrive.getCurrentPosition() - (int) (inches * countsPerInch));

        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);
        leftBackDrive.setTargetPosition(newLeftBackTarget);
        rightBackDrive.setTargetPosition(newRightBackTarget);

        // Turn On RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double sensorReading = Double.MAX_VALUE;
        runtime.reset();
        leftDrive.setPower(-speed);
        rightDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(-speed);
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.04, 0.00000000008, 0.000001);
        //theOpMode.telemetry.setMsTransmissionInterval(50);
        double degreeCount = 0;
        runtime.reset();
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while ((Math.abs(targetAngle - getAbsoluteAngle()) > 1 || pid.getLastSlope() > 1.25 || degreeCount < 4) && runtime.seconds() < timeoutS) {
            if (Math.abs(targetAngle - getAbsoluteAngle()) < 1){
                degreeCount +=1;
            }
            double motorSpeed = pid.update(getAbsoluteAngle());
            leftDrive.setPower(-motorSpeed);
            rightDrive.setPower(motorSpeed);
            leftBackDrive.setPower(motorSpeed);
            rightBackDrive.setPower(-motorSpeed);
            theOpMode.telemetry.addData("degreeCount", degreeCount);
            theOpMode.telemetry.addData("Current Angle", getAbsoluteAngle());
            theOpMode.telemetry.addData("Target Angle", targetAngle);
            theOpMode.telemetry.addData("Slope", pid.getLastSlope());
            theOpMode.telemetry.addData("Power", motorSpeed);
            theOpMode.telemetry.update();
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }






    public void DriverControls() {
        if (theOpMode.gamepad1.y){
            upFlag = true;
        } else {
            upFlag = false;
            upPersistent = false;
        }
        if (upFlag && !upPersistent) {
            if (drvTrnSpd < 1){drvTrnSpd += .2;}
            upPersistent = true;
        }
        if (theOpMode.gamepad1.a){
            downFlag = true;
        } else {
            downFlag = false;
            downPersistent = false;
        }
        if (downFlag && !downPersistent) {
            if (drvTrnSpd > .1){drvTrnSpd -= .2;}
            downPersistent = true;
        }


    }
    public void gyroLift(double speed, double pos, double timeoutS) {


    }
    public void UpdateDriveTrain() {
        drvTrnSpd = 1;
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        AbsoluteValue = -imuCH.getAngularOrientation().firstAngle;
        if (theOpMode.gamepad1.b) {
            ZeroPosition = AbsoluteValue;

        }
        double y = theOpMode.gamepad1.left_stick_y; // Remember, this is reversed!
        double x = -theOpMode.gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = -theOpMode.gamepad1.right_stick_x;

        // Read inverse IMU heading, as the IMU heading is CW positive

        //double botHeading = AbsoluteValue-ZeroPosition;
        double botHeading = AbsoluteValue-ZeroPosition;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        leftDrive.setPower(frontLeftPower*drvTrnSpd);
        leftBackDrive.setPower(backLeftPower*drvTrnSpd);
        rightDrive.setPower(frontRightPower*drvTrnSpd);
        rightBackDrive.setPower(backRightPower*drvTrnSpd);


    }
    public void testDrivetrain() {
        leftDrive.setPower(theOpMode.gamepad1.left_stick_y);
        rightDrive.setPower(theOpMode.gamepad1.right_stick_y);
        leftBackDrive.setPower(theOpMode.gamepad1.left_trigger);
        rightBackDrive.setPower(theOpMode.gamepad1.right_trigger);

    }
    public void testDrivetrainAuto() {
        while (((LinearOpMode) theOpMode).opModeIsActive()) {
            rightBackDrive.setPower(.5);
        }
    }

      /*public void ResetHeading() {
          if (theOpMode.gamepad1.b) {
              angles = imu.getAngularOrientation
                      (AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
              IMUReading = (-imu.getAngularOrientation().firstAngle);
              startingHeadingInRadians = (Math.toRadians(IMUReading) + 3.14159);
              theOpMode.telemetry.addData("Starting heading", startingHeadingInRadians);
              theOpMode.telemetry.addData("IMUReading", IMUReading);
              theOpMode.telemetry.update();


          }
      }*/
    // https://www.youtube.com/watch?v=xm3YgoEiEDc

}
