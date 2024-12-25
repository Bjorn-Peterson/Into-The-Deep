package org.firstinspires.ftc.teamcode.NewRobot;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.NewRobot.Lift;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;



@Config
@Autonomous(name = "Specimen", group = "Autonomous")
public class LeftAuto extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(50, -10, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);


        Action preload = drive.actionBuilder(initialPose).
                setTangent(0).
                strafeTo(new Vector2d(40, 0)).
                build();

                Action score2 = drive.actionBuilder(new Pose2d(12, 25, Math.toRadians(-27))).
                turn(Math.toRadians(27)).
                build();

        waitForStart();

        Actions.runBlocking( new ParallelAction(score2, pidf.collectRun(), lift.liftDown()));
              //  new ParallelAction(afterSub, lift.liftAction()));
    }
}