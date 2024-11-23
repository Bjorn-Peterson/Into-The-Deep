package org.firstinspires.ftc.teamcode.Ri3d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.OldStuff.Collection3d;
import org.firstinspires.ftc.teamcode.OldStuff.OldLift;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;
import org.firstinspires.ftc.teamcode.OldStuff.ri3d;
@Autonomous
public class RightAuto extends LinearOpMode {

    public void runOpMode() {
        OldLift oldLift = new OldLift(hardwareMap, this, 145.1 , 1, 1.2);
        ri3d ri3d = new ri3d(hardwareMap, this, 384.5, 1, 4);
        Collection3d collection = new Collection3d(hardwareMap, this);
        PIDF pidf = new PIDF(hardwareMap, this);


        waitForStart();
        collection.arm(.2, 2, .9);
        sleep(600);
        collection.colState(Collection3d.collectionState.OPEN);
        sleep(800);
        collection.arm(-.3, -100, 0.9);
        //ri3d.strafeEncoderDrive(.6, 40, 3);




    }

}