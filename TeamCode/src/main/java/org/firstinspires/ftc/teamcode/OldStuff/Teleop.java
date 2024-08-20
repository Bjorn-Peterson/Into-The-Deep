
package org.firstinspires.ftc.teamcode.OldStuff;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

//https:www.youtube.com/watch?v=pQ_aVTM9qX0
@TeleOp(name = "Trash Worst Robot In existence we should quit", group = "Iterative Opmode")
public class Teleop extends OpMode {
    Drivetrain drivetrain;
    Lift lift;
    Collection collection;
    ArrayList<Double> liftHeights = new ArrayList<Double>();
    DistanceSensor distanceSensor;
    //PIDFLift pidfLift;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, this, 537.6, 1.0, 4.0);
        lift = new Lift(hardwareMap, this, 537.6, 1, 2, liftHeights);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "deliveryDistanceSensor");
        //pidfLift = new PIDFLift(hardwareMap, this, 145.1, 1.0, 2.0);


        //test = new Test(hardwareMap, this);
        liftHeights.add(1.0);
        liftHeights.add(7.0);
        liftHeights.add(15.0);
        liftHeights.add(16.1);
        liftHeights.add(1.8);
        liftHeights.add(16.1);
        collection = new Collection(hardwareMap, this);

        //  armAndClaw = new Arm_and_Claw(hardwareMap, this);

    }
    @Override
    public void loop() {
        drivetrain.UpdateDriveTrain();
        //  lift.raiseLiftToDesignatedPositionTeleop();
        drivetrain.DriverControls();
        //   rubberBandSpinner.rotateSpinnersTeleop();
        //  drivetrain.testDrivetrain();
        collection.collectionTeleop();
        collection.setCollectionPosition();
        //pidfLift.teleLift();
        lift.teleLift();

        /*
        if (collection.imAboutToDie.getPosition() > .7 && distanceSensor.getDistance(DistanceUnit.INCH) <= 8 && distanceSensor.getDistance(DistanceUnit.INCH) >= 6) {
            collection.imAboutToDie.setPosition(.9);
            collection.imGoingToDie.setPosition(.9);
        }
        else if (collection.imAboutToDie.getPosition() > .7 && distanceSensor.getDistance(DistanceUnit.INCH) < 6) {
            collection.imAboutToDie.setPosition(.8);
            collection.imGoingToDie.setPosition(.8);
        }
        else if (collection.imAboutToDie.getPosition() > .7 && distanceSensor.getDistance(DistanceUnit.INCH) > 8) {
            collection.imAboutToDie.setPosition(.8);
            collection.imGoingToDie.setPosition(.8);
        }
        telemetry.addData("Current Distance", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
         */

    }


    @Override
    public void stop() {
    }

}
