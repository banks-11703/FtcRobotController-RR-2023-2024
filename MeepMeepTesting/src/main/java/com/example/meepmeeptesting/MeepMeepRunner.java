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

        final Pose2d startBlueFront = new Pose2d(-36, 64-20, Math.toRadians(90));
        final Pose2d startRedFront = new Pose2d(-36, -64-20, Math.toRadians(-90));
        final Pose2d startBlueBack = new Pose2d(12, 64-20, Math.toRadians(90));
        final Pose2d startRedBack = new Pose2d(12, -64-20, Math.toRadians(-90));
        final Pose2d backdropPos1 = new Pose2d(59.5, 42, 0);
        final Pose2d backdropPos2 = new Pose2d(59.5, 35, 0);
        final Pose2d backdropPos3 = new Pose2d(59.5, 27, 0);
        final Pose2d backdropPos4 = new Pose2d(59.5, -29, 0);
        final Pose2d backdropPos5 = new Pose2d(59.5, -36, 0);
        final Pose2d backdropPos6 = new Pose2d(59.5, -42.75, 0);
        final Vector2d spikePosRedFront = new Vector2d(-36, -36);
        final Vector2d spikePosBlueFront = new Vector2d(-36, 36-20);
        final Vector2d spikePosRedBack = new Vector2d(12 + 8, -38);
        final Vector2d spikePosBlueBack = new Vector2d(12 + 8, 31.5);
        final Pose2d stackPosRedClose = new Pose2d(-62, -36, Math.toRadians(0));
        final Pose2d stackPosRedMedium = new Pose2d(-62, -24, Math.toRadians(0));
        final Pose2d stackPosRedFar = new Pose2d(-62, -12, Math.toRadians(0));
        final Pose2d stackPosBlueFar = new Pose2d(-62, 12, Math.toRadians(0));
        final Pose2d stackPosBlueMedium = new Pose2d(-62, 24, Math.toRadians(0));
        final Pose2d stackPosBlueClose = new Pose2d(-62, 36-20, Math.toRadians(0));
        final Vector2d parkCloseRed = new Vector2d(60, -60);
        final Vector2d parkFarRed = new Vector2d(60, -12);
        final Vector2d parkCloseBlue = new Vector2d(60, 60);
        final Vector2d parkFarBlue = new Vector2d(60, 12-20);

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
                .setStartPose(startRedFront)
                .setStartPose(new Pose2d(-48, -48, -Math.PI / 2))
                .build();


        int randomizationResult = 1;
        int yMod = -1;
        double headingMod = 0;
        int park = 0;
        blueFrontBot.runAction(blueFrontBot.getDrive().actionBuilder(blueFrontBot.getPose())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(spikePosRedFront.plus(new Vector2d(4, 5.5)), Math.PI), 0)
//                .afterTime(0, intakeTraj(0, drive))
//                .afterTime(0, purplePixelTraj(grabyl_out, grabyr_out, drive))
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(spikePosRedFront.plus(new Vector2d(0, 5.5)), Math.PI), Math.PI)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.00f)
//                .addEntity(blueBackBot)
                .addEntity(blueFrontBot)
//                .addEntity(redBackBot)
//                .addEntity(redFrontBot)
                .start();
    }
}