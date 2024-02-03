package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRunner {
    public static void main(String[] args) {

        final Pose2d startBlueFront = new Pose2d(-36, 64, Math.toRadians(90));
        final Pose2d startRedFront = new Pose2d(-36, -64, Math.toRadians(-90));
        final Pose2d startBlueBack = new Pose2d(12, 64, Math.toRadians(90));
        final Pose2d startRedBack = new Pose2d(12, -64, Math.toRadians(-90));
        Pose2d finalStart = new Pose2d(0, 0, 0);
        final Pose2d backdropPos1 = new Pose2d(62, 41, 0);
        final Pose2d backdropPos2 = new Pose2d(62, 35, 0);
        final Pose2d backdropPos3 = new Pose2d(62, 29, 0);
        final Pose2d backdropPos4 = new Pose2d(62, -29, 0);
        final Pose2d backdropPos5 = new Pose2d(62, -35, 0);
        final Pose2d backdropPos6 = new Pose2d(62, -41, 0);
        final Vector2d spikePosRedFront = new Vector2d(-36, -36);
        final Vector2d spikePosBlueFront = new Vector2d(-36, 36);
        final Vector2d spikePosRedBack = new Vector2d(12 + 8, -38);
        final Vector2d spikePosBlueBack = new Vector2d(12 + 8, 31.5);

        final Vector2d parkCloseRed = new Vector2d(60, -60);
        final Vector2d parkFarRed = new Vector2d(60, -12);
        final Vector2d parkCloseBlue = new Vector2d(60, 60);
        final Vector2d parkFarBlue = new Vector2d(60, 12);

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity blueBackBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeRedDark())
                .setStartPose(startBlueBack)
                .build();
        RoadRunnerBotEntity blueFrontBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeRedLight())
                .setStartPose(startBlueFront)
                .build();
        RoadRunnerBotEntity redBackBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeBlueDark())
                .setStartPose(startRedBack)
                .build();
        RoadRunnerBotEntity redFrontBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeBlueLight())
//                .setStartPose(startRedFront)
                .setStartPose(new Pose2d(-48, -48, -Math.PI / 2))
                .build();

        int rand = 0;
//        redFrontBot.runAction(redFrontBot.getDrive().actionBuilder(redFrontBot.getPose())
//                .splineToSplineHeading(new Pose2d(new Vector2d(-24, -60), 0), 0)
//                .splineToSplineHeading(new Pose2d(new Vector2d(36, -60), Math.toRadians(0)), 0)
//                .build()
//        );
        blueFrontBot.runAction(blueFrontBot.getDrive().actionBuilder(blueFrontBot.getPose())
//                .waitSeconds(5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(spikePosBlueFront.plus(new Vector2d(0, 2)), Math.PI / 2), Math.PI / 2)
                .waitSeconds(1)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(new Vector2d(-28, 60), 0), 0)
                .splineToSplineHeading(new Pose2d(new Vector2d(12, 60), Math.toRadians(0)), 0)
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(24, 52, -Math.PI / 6), -Math.PI / 6)
                .splineToSplineHeading(new Pose2d(backdropPos2.position.plus(new Vector2d(-3, 1)), 0), 0)
                .waitSeconds(1)
                .strafeToLinearHeading(backdropPos2.position.plus(new Vector2d(-18, 1)), 0)
                .splineToSplineHeading(new Pose2d(parkCloseBlue.plus(new Vector2d(-12, 0)), 0), 0)
                .splineToSplineHeading(new Pose2d(parkCloseBlue, 0), 0)
                .build()
        );

        redFrontBot.runAction(redFrontBot.getDrive().actionBuilder(redFrontBot.getPose())
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(blueBackBot)
                .addEntity(blueFrontBot)
//                .addEntity(redBackBot)
//                .addEntity(redFrontBot)
                .start();
    }
}