package org.firstinspires.ftc.teamcode.OldStuff;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.NewRobot.Drivetrain;

//https:www.youtube.com/watch?v=pQ_aVTM9qX0
@TeleOp(name = "pdflTele", group = "Iterative Opmode")
public class PDFLTeleop extends OpMode {
    Drivetrain drivetrain;
    Collection3d collection3d;
    PDFL pdfl;
    PIDF pidf;
    TurnPIDController pidController;

    //PIDFLift pidfLift;



    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, this, 384.5, 1, 4.09);
      //  delivery = new Delivery(hardwareMap, this);
        pdfl = new PDFL(1.1, 0, 0, 0, hardwareMap,this,145.1, 1,1);
        pidController = new TurnPIDController(hardwareMap, this,0.1,0,0, 145.1, 1, 1);
        //collection3d = new Collection3d(hardwareMap, this);




    }
    @Override
    public void loop() {
        drivetrain.UpdateDriveTrain();

      // pidController.update();

       pdfl.run(1);



      //  collection3d.teleopControls();





    }


    @Override
    public void stop() {
    }

}
