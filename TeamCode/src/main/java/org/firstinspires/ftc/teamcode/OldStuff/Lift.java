package org.firstinspires.ftc.teamcode.OldStuff;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;

public class Lift {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DistanceSensor sensorRange;
    private ElapsedTime runtime = new ElapsedTime();
    private OpMode theOpMode;
    private int teleopLiftHeight = 0;
    private double targetHeight = 0;
    double countsPerInch;
    private double kp = 0.1;
    private double ki = 0.01;
    private double kd = 0.001;

    // Variables for PID control
    private double integral = 0;
    private double previousError = 0;
    // private TouchSensor sensorTouch;



    public Lift(HardwareMap hardwareMap, OpMode opMode, double encoderTicksPerRev, double gearRatio, double wheelDiameter) {
        countsPerInch = (encoderTicksPerRev * gearRatio) / (wheelDiameter * 3.14);
        theOpMode = opMode;
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        //   sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // sensorTouch = hardwareMap.get(TouchSensor.class, "touchSensor");
    }
    public void teleLift() {

        if (theOpMode.gamepad2.right_trigger > .05) {
            leftMotor.setPower(-theOpMode.gamepad2.right_trigger);
            rightMotor.setPower(theOpMode.gamepad2.right_trigger);

        }
        else if (theOpMode.gamepad2.left_trigger > .05) {
            leftMotor.setPower(theOpMode.gamepad2.left_trigger);
            rightMotor.setPower(-theOpMode.gamepad2.left_trigger);

        }
        else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
    }
    public void soloControls(){

            if (theOpMode.gamepad1.right_trigger > .05) {
                leftMotor.setPower(-theOpMode.gamepad1.right_trigger);
                rightMotor.setPower(theOpMode.gamepad1.right_trigger);

            }
            else if (theOpMode.gamepad1.left_trigger > .05) {
                leftMotor.setPower(theOpMode.gamepad1.left_trigger);
                rightMotor.setPower(-theOpMode.gamepad1.left_trigger);

            }
            else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }
        }


    public void liftAuto(double power, int target, double timeoutS) {
        runtime.reset();
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        leftMotor.setTargetPosition(target);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (((LinearOpMode) theOpMode).opModeIsActive() && runtime.seconds() < timeoutS && leftMotor.isBusy()) {
//            leftMotor.setPower(power);
//            leftMotor.setTargetPosition(leftMotor.getCurrentPosition());
//            theOpMode.telemetry.addData("power", power);
//            theOpMode.telemetry.update();

        }
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void liftAutoStart(double power, int target, double timeoutS) {
        runtime.reset();
        leftMotor.setPower(power);
        rightMotor.setPower(power);
        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public boolean liftAutoCheck(double power, int target, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() &&
                (runtime.seconds() < timeoutS) && (leftMotor.isBusy() && (rightMotor.isBusy()))) {
            theOpMode.telemetry.addData("Current Pos", leftMotor.getCurrentPosition());
            theOpMode.telemetry.addData("Current Pos", rightMotor.getCurrentPosition());
            theOpMode.telemetry.addData("Target Pos", target);
            theOpMode.telemetry.update();
            return true;
        }
        else {
            liftAutoEnd();
            return false;
        }
    }

    public void liftAutoEnd() {
        //  if (sensorTouch.isPressed()) {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //  }
    }







/*


    public void raiseLiftToDesignatedPosition(int designatedPosition, int timeoutS) {

        runtime.reset();
        while (((LinearOpMode) theOpMode).opModeIsActive() &&
                Math.abs(liftHeights.get(designatedPosition) - (sensorRange.getDistance(DistanceUnit.INCH))) > .9 &&
                (runtime.seconds() < timeoutS)) {
            if ((liftHeights.get(designatedPosition)) > (sensorRange.getDistance(DistanceUnit.INCH))) {
                double rangeErrorOfLiftHeight = liftHeights.get(designatedPosition) - sensorRange.getDistance(DistanceUnit.INCH);
                double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 3);
                leftMotor.setPower(speedOfLiftForDesignatedPosition);
                rightMotor.setPower(speedOfLiftForDesignatedPosition);

            } else {

                double rangeErrorOfLiftHeight = liftHeights.get(designatedPosition) - sensorRange.getDistance(DistanceUnit.INCH);
                double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 3);
                rightMotor.setPower(speedOfLiftForDesignatedPosition);
                leftMotor.setPower(speedOfLiftForDesignatedPosition);
            }
        }
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    public void raiseLiftToDesignatedPositionStart(int designatedPosition, double timeoutS) {
        runtime.reset();
        double rangeErrorOfLiftHeight = liftHeights.get(designatedPosition) - sensorRange.getDistance(DistanceUnit.INCH);
        double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 3);
        leftMotor.setPower(speedOfLiftForDesignatedPosition);
        rightMotor.setPower(speedOfLiftForDesignatedPosition);

    }

    public boolean raiseLiftToDesignatedPositionCheck(int designatedPosition, double timeoutS) {
        if (((LinearOpMode) theOpMode).opModeIsActive() &&
                Math.abs(liftHeights.get(designatedPosition) - (sensorRange.getDistance(DistanceUnit.INCH))) > 1.5 &&
                (runtime.seconds() < timeoutS)) {
            double rangeErrorOfLiftHeight = liftHeights.get(designatedPosition) - sensorRange.getDistance(DistanceUnit.INCH);
            double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 3);
            leftMotor.setPower(speedOfLiftForDesignatedPosition);
            rightMotor.setPower(speedOfLiftForDesignatedPosition);
            theOpMode.telemetry.addData("Current Position", sensorRange.getDistance(DistanceUnit.INCH));
            theOpMode.telemetry.update();
            return true;
        } else {
            raiseLiftToDesignatedPositionEnd();
            return false;
        }
    }

    public void raiseLiftToDesignatedPositionTest(int designatedPosition, double timeoutS) {
        raiseLiftToDesignatedPositionStart(designatedPosition, timeoutS);
        while (raiseLiftToDesignatedPositionCheck(designatedPosition, timeoutS)) ;
    }

    public void raiseLiftToDesignatedPositionEnd() {
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    public void raiseLiftToDesignatedPositionTeleop() {
        theOpMode.telemetry.addData("Running to", " %.01f", targetHeight);
        theOpMode.telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
        theOpMode.telemetry.update();
        if (theOpMode.gamepad2.b) {
            teleopLiftHeight = 3;
            targetHeight = liftHeights.get(teleopLiftHeight);
        }
        if (theOpMode.gamepad2.a) {
            teleopLiftHeight = 0;
            targetHeight = liftHeights.get(teleopLiftHeight);
        }
        if (theOpMode.gamepad2.x) {
            teleopLiftHeight = 1;
            targetHeight = liftHeights.get(teleopLiftHeight);
        }
        if (theOpMode.gamepad2.y) {
            teleopLiftHeight = 2;
            targetHeight = liftHeights.get(teleopLiftHeight);

        }
        if (theOpMode.gamepad2.dpad_up) {
            teleopLiftHeight = 4;
            targetHeight = liftHeights.get(teleopLiftHeight);

        }
        if (theOpMode.gamepad2.dpad_down) {
            teleopLiftHeight = 5;
            targetHeight =
                    liftHeights.get(teleopLiftHeight);
        }

        if (Math.abs(theOpMode.gamepad2.right_stick_y) > .1) {
            targetHeight = sensorRange.getDistance(DistanceUnit.INCH) - (theOpMode.gamepad2.right_stick_y * 3);
            theOpMode.telemetry.addData("Running to", " %.01f", targetHeight);
            theOpMode.telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
            theOpMode.telemetry.update();
        }

        // Moving lift up
//        if ((targetHeight) - (sensorRange.getDistance(DistanceUnit.INCH)) > 1.2 ||
//                ((targetHeight) - (sensorRange.getDistance(DistanceUnit.INCH)) < -1.2)) {
//            theOpMode.telemetry.addData("Moving", "up");
//            theOpMode.telemetry.update();
        if ((targetHeight) > (sensorRange.getDistance(DistanceUnit.INCH))) {
            double rangeErrorOfLiftHeight = (targetHeight) - sensorRange.getDistance(DistanceUnit.INCH);
            double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 6.69);
            speedOfLiftForDesignatedPosition = Math.max(-.2, Math.min(1, speedOfLiftForDesignatedPosition));
            rightMotor.setPower(speedOfLiftForDesignatedPosition);
            leftMotor.setPower(speedOfLiftForDesignatedPosition);

            //theOpMode.telemetry.addData("range", String.format("%.2f", speedOfLiftForDesignatedPosition));
            //theOpMode.telemetry.update();

        } else {
            //Moving lift down
            //    theOpMode.telemetry.addData("Moving", "down");
            double rangeErrorOfLiftHeight = (targetHeight) - sensorRange.getDistance(DistanceUnit.INCH);
            double speedOfLiftForDesignatedPosition = (rangeErrorOfLiftHeight / 6.69);
            speedOfLiftForDesignatedPosition = Math.max(-1, Math.min(1, speedOfLiftForDesignatedPosition));
            /*
            theOpMode.telemetry.addData("Error", rangeErrorOfLiftHeight);
            theOpMode.telemetry.addData("Speed", speedOfLiftForDesignatedPosition);
            theOpMode.telemetry.addData("Pos", rightMotor.getCurrentPosition());
            theOpMode.telemetry.update();



            // if (rightMotor.getCurrentPosition() > 0) {

            rightMotor.setPower(speedOfLiftForDesignatedPosition);
            leftMotor.setPower(speedOfLiftForDesignatedPosition);

//                } else {
//                    rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                }

        }

    }

    public final boolean opModeIsActive() {
        return false;
    }



    public void runOpMode () {

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;
        theOpMode.telemetry.addData(">>", "Press start to continue");
        theOpMode.telemetry.update();

        while (opModeIsActive()) {
            // generic DistanceSensor methods.
            theOpMode.telemetry.addData("deviceName", sensorRange.getDeviceName());
            theOpMode.telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
            // Rev2mDistanceSensor specific methods.
            theOpMode.telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            theOpMode.telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
            theOpMode.telemetry.update();
        }

*/
}
