package org.firstinspires.ftc.teamcode.OldStuff;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleopTest")
public class Teleop extends OpMode {

    PIDF pidf;

    //PIDFLift pidfLift;



    @Override
    public void init() {
      pidf = new PIDF(hardwareMap, this);
    }
    @Override
    public void loop() {

        pidf.testColor(false);

    }


    @Override
    public void stop() {
    }

}
