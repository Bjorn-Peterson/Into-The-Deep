package org.firstinspires.ftc.teamcode.OldStuff;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.NewRobot.Lift;

//https:www.youtube.com/watch?v=pQ_aVTM9qX0
@TeleOp(name = "SoloTele", group = "Iterative Opmode")
public class SoloTeleop extends OpMode {
    ri3d ri3d;
    Collection3d collection3d;
    OldLift oldLift;

    //PIDFLift pidfLift;



    @Override
    public void init() {
        ri3d = new ri3d(hardwareMap, this, 537.6, 1.0, 4.0);
        collection3d = new Collection3d(hardwareMap, this);
        oldLift = new OldLift(hardwareMap, this, 537.6, 1, 2);
        // pdfl = new PDFL(0,0,0,0, 384.5, 1, 1);


    }
    @Override
    public void loop() {
        ri3d.UpdateDriveTrain();
        collection3d.teleopControls();
        //lift.teleLift();
        //collection3d = new Collection3d(hardwareMap, this);
       // pidf.tele();
        oldLift.soloControls();


    }


    @Override
    public void stop() {
    }

}
