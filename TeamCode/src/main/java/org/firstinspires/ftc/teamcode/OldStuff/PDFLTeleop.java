package org.firstinspires.ftc.teamcode.OldStuff;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//https:www.youtube.com/watch?v=pQ_aVTM9qX0
@TeleOp(name = "pdflTele", group = "Iterative Opmode")
public class PDFLTeleop extends OpMode {
    ri3d ri3d;
    Collection3d collection3d;
    Lift lift;
    PDFL pdfl;

    //PIDFLift pidfLift;



    @Override
    public void init() {
        ri3d = new ri3d(hardwareMap, this, 384.5, 1.0, 4.0);
        pdfl = new PDFL(.1, .1, .1, .1, hardwareMap,this,537.6, 1,1);


    }
    @Override
    public void loop() {
        ri3d.UpdateDriveTrain();
        pdfl.run(3);
       // ri3d.DriverControls();





    }


    @Override
    public void stop() {
    }

}
