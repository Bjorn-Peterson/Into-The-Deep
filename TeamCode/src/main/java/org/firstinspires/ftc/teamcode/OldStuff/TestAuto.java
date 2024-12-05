package org.firstinspires.ftc.teamcode.OldStuff;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.NewRobot.Drivetrain;
import org.firstinspires.ftc.teamcode.NewRobot.Lift;

//https:www.youtube.com/watch?v=pQ_aVTM9qX0
@Autonomous
public class TestAuto extends LinearOpMode {
    Drivetrain drivetrain;
    Collection3d collection3d;
    PDFL pdfl;
    PIDF pidf;
    Lift lift;

    //PIDFLift pidfLift;



    @Override
    public void runOpMode() {
        drivetrain = new Drivetrain(hardwareMap, this, 384.5, 1, 4.09);
      //  delivery = new Delivery(hardwareMap, this);
       // pdfl = new PDFL(hardwareMap,this);
        pidf = new PIDF(hardwareMap, this);
        //collection3d = new Collection3d(hardwareMap, this);
        lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);

        waitForStart();
        pidf.auto(300);



    }

}
