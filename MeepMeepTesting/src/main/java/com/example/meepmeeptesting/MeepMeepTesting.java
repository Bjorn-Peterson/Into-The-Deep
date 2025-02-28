package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 12)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(40, 0, Math.toRadians(90))).
                strafeToSplineHeading(new Vector2d(-6, 32.5), Math.toRadians(-90)).

                build());



        RoadRunnerBotEntity bot2 = new DefaultBotBuilder(meepMeep).setConstraints(80, 80, Math.toRadians(180), Math.toRadians(180), 12)
                .build();
                bot2.runAction(bot2.getDrive().actionBuilder(new Pose2d(0,0,Math.toRadians(-90))).
                        waitSeconds(2).
                        strafeTo(new Vector2d(12.5, -47)).
                strafeTo(new Vector2d(12.5, -56)).
                waitSeconds(.25).

                turn(Math.toRadians(-35)).
                waitSeconds(.25).

                waitSeconds(.4).
                strafeTo(new Vector2d(1, -30)).

                strafeTo(new Vector2d(31.5, 8)).

                strafeTo(new Vector2d(1, -30)).
                build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
              //  .addEntity(myBot)
                .addEntity(myBot)
                .start();
    }
}
