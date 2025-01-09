package org.firstinspires.ftc.teamcode.OldStuff;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//https:www.youtube.com/watch?v=pQ_aVTM9qX0
@TeleOp(name = "Teleop", group = "Iterative Opmode")
public class Teleop extends OpMode {

    PIDF pidf;

    //PIDFLift pidfLift;



    @Override
    public void init() {
      pidf = new PIDF(hardwareMap, this);
    }
    @Override
    public void loop() {

        pidf.testColor(true);

    }


    @Override
    public void stop() {
    }

}
