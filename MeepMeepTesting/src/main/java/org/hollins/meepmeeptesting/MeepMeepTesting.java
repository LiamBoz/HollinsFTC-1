package org.hollins.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting{
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setDriveTrainType(DriveTrainType.MECANUM)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(88, 88, Math.toRadians(480), Math.toRadians(480), 11.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(270)))
/*
                                .splineTo(new Vector2d(0,-30), Math.toRadians(270))
                                .splineTo(new Vector2d(0,-49), Math.toRadians(360))
                                .setReversed(true)
                                .splineTo(new Vector2d(-26, -49), Math.toRadians(180))*/
                                .splineToLinearHeading(new Pose2d(2,-49, Math.toRadians(180)), Math.toRadians(270))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(70, -49, Math.toRadians(180)), Math.toRadians(180))
                                /*        .splineToLinearHeading(new Pose2d(0,-15, Math.toRadians(270)), Math.toRadians(270))
                                .splineToLinearHeading(new Pose2d(0,-49, Math.toRadians(360)), Math.toRadians(270))
                                //.setReversed(true)
                                .splineToConstantHeading(new Vector2d(-26, -49), Math.toRadians(180))*/
                                .build()
                );
        // Declare out second bot
//        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
//                // We set this bot to be red
//                .setColorScheme(new ColorSchemeRedDark())
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(30, 30, Math.toRadians(180)))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .forward(30)
//                                .turn(Math.toRadians(90))
//                                .build()
//                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)

                // Add both of our declared bot entities
                .addEntity(myBot)
                //.addEntity(mySecondBot)
                .start();

    }
}