package org.firstinspires.ftc.teamcode.OldStuff;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;


import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

//https:www.youtube.com/watch?v=pQ_aVTM9qX0
@TeleOp(name = "Tele", group = "Iterative Opmode")
public class BasicTeleop extends OpMode {
    ri3d ri3d;
    Collection3d collection3d;
    Lift lift;
    PDFL pdfl;

    //PIDFLift pidfLift;



    @Override
    public void init() {
        ri3d = new ri3d(hardwareMap, this, 537.6, 1.0, 4.0);
        collection3d = new Collection3d(hardwareMap, this);
        lift = new Lift(hardwareMap, this, 537.6, 1, 2);
        pdfl = new PDFL(0,0,0,0, 384.5, 1, 1);


    }
    @Override
    public void loop() {
        ri3d.UpdateDriveTrain();
       // ri3d.DriverControls();
        collection3d.teleopControls();
        lift.teleLift();





    }


    @Override
    public void stop() {
    }

}
