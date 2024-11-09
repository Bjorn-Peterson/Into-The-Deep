package org.firstinspires.ftc.teamcode.Ri3d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OldStuff.Collection3d;
import org.firstinspires.ftc.teamcode.OldStuff.Lift;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;
import org.firstinspires.ftc.teamcode.OldStuff.ri3d;
@Autonomous
public class RightAuto extends LinearOpMode {

    public void runOpMode() {
        Lift lift = new Lift(hardwareMap, this, 145.1 , 1, 1.2);
        ri3d ri3d = new ri3d(hardwareMap, this, 384.5, 1, 4.09449);
        Collection3d collection = new Collection3d(hardwareMap, this);
        PIDF pidf = new PIDF(hardwareMap, this);


        waitForStart();
        ri3d.encoderDrive(.5, 15, 2);
        ri3d.strafeEncoderDrive(.5, -15, 2);

    }

}