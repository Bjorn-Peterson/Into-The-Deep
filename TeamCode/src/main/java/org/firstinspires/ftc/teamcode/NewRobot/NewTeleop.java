package org.firstinspires.ftc.teamcode.NewRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OldStuff.PIDF;


//https:www.youtube.com/watch?v=pQ_aVTM9qX0
@TeleOp
public class NewTeleop extends OpMode {

    PIDF pidf;
    Drivetrain drivetrain;

    Lift lift;



    @Override
    public void init() {
        lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);
        pidf = new PIDF(hardwareMap, this);
        drivetrain = new Drivetrain(hardwareMap, this, 384.5, 1, 4);


    }
    @Override
    public void loop() {
        drivetrain.UpdateDriveTrain();
        lift.soloControls();
         pidf.tele();
         //lift.teleLift();
        //pidf.test();


    }


    @Override
    public void stop() {
    }

}
