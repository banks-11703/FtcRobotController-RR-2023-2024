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
        final Pose2d backdropPos1 = new Pose2d(59.5, 42, 0);
        final Pose2d backdropPos2 = new Pose2d(59.5, 35, 0);
        final Pose2d backdropPos3 = new Pose2d(59.5, 27, 0);
        final Pose2d backdropPos4 = new Pose2d(59.5, -29, 0);
        final Pose2d backdropPos5 = new Pose2d(59.5, -36, 0);
        final Pose2d backdropPos6 = new Pose2d(59.5, -42.75, 0);
        final Vector2d spikePosRedFront = new Vector2d(-36, -36);
        final Vector2d spikePosBlueFront = new Vector2d(-36, 36);
        final Vector2d spikePosRedBack = new Vector2d(12 + 8, -38);
        final Vector2d spikePosBlueBack = new Vector2d(12 + 8, 31.5);
        final Pose2d stackPosRedClose = new Pose2d(-62, -36, Math.toRadians(0));
        final Pose2d stackPosRedMedium = new Pose2d(-62, -24, Math.toRadians(0));
        final Pose2d stackPosRedFar = new Pose2d(-62, -12, Math.toRadians(0));
        final Pose2d stackPosBlueFar = new Pose2d(-62, 12, Math.toRadians(0));
        final Pose2d stackPosBlueMedium = new Pose2d(-62, 24, Math.toRadians(0));
        final Pose2d stackPosBlueClose = new Pose2d(-62, 36, Math.toRadians(0));
        final Vector2d parkCloseRed = new Vector2d(60, -60);
        final Vector2d parkFarRed = new Vector2d(60, -12);
        final Vector2d parkCloseBlue = new Vector2d(60, 60);
        final Vector2d parkFarBlue = new Vector2d(60, 12);

        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity blueBackBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeRedDark())
                .setStartPose(startBlueBack)
                .build();
        RoadRunnerBotEntity blueBackBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeRedDark())
                .setStartPose(startBlueBack)
                .build();
        RoadRunnerBotEntity blueBackBot3 = new DefaultBotBuilder(meepMeep)
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
        RoadRunnerBotEntity redFrontBot1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(startRedFront)
                .build();
        RoadRunnerBotEntity redFrontBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(startRedFront)
                .build();
        RoadRunnerBotEntity redFrontBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 13)
                .setDimensions(15, 16)
                .setColorScheme(new ColorSchemeBlueLight())
                .setStartPose(startRedFront)
                .build();


        int randomizationResult = 1;
        int yMod = -1;
        double headingMod = 0;
        int park = 0;
        redFrontBot1.runAction(redFrontBot1.getDrive().actionBuilder(redFrontBot1.getPose())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(spikePosRedFront.plus(new Vector2d(3, 5.5)), Math.PI), 0)
                .waitSeconds(1.9)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(spikePosRedFront.plus(new Vector2d(0, 5.5)), Math.PI), Math.PI)
                .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(9, 0)), 0)
                .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(2.5, 0)), Math.toRadians(-7))
                .waitSeconds(1)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-40, -60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12, -60, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(backdropPos5.position.plus(new Vector2d(-6.5, 0)), Math.toRadians(0)), 0)
                .strafeToLinearHeading(backdropPos5.position.plus(new Vector2d(-20, 0)), 0)
                .splineToSplineHeading(new Pose2d(parkFarRed.plus(new Vector2d(-12, 0)), 0), 0)
                .splineToSplineHeading(new Pose2d(parkFarRed.plus(new Vector2d(0, 0)), 0), 0)
                .build()
        );

        redFrontBot2.runAction(redFrontBot2.getDrive().actionBuilder(redFrontBot2.getPose())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(spikePosRedFront.plus(new Vector2d(-3, 5.5)), 0), Math.PI)
                .waitSeconds(1)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(spikePosRedFront.plus(new Vector2d(0, 5.5)), 0), 0)
                .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(9, 0)), 0)
                .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(2.5, 0)), Math.toRadians(-7))
                .waitSeconds(1)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-40, -60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12, -60, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(2.0)
                .splineToSplineHeading(new Pose2d(backdropPos4.position.plus(new Vector2d(-6.5, 0)), Math.toRadians(0)), 0)
                .strafeToLinearHeading(backdropPos4.position.plus(new Vector2d(-20, 0)), 0)
                .splineToSplineHeading(new Pose2d(parkFarRed.plus(new Vector2d(-12, 0)), 0), 0)
                .splineToSplineHeading(new Pose2d(parkFarRed.plus(new Vector2d(0, 0)), 0), 0)
                .build()
        );

        redFrontBot3.runAction(redFrontBot3.getDrive().actionBuilder(redFrontBot3.getPose())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(spikePosRedFront.plus(new Vector2d(0, 7)), -Math.PI / 2), -Math.PI / 2)
                .waitSeconds(2.2)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(spikePosRedFront.plus(new Vector2d(0, 5.5)), -Math.PI / 2), -Math.PI / 2)
                .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(9, 0)), 0)
                .strafeToLinearHeading(stackPosRedClose.position.plus(new Vector2d(2.5, 0)), Math.toRadians(-7))
                .waitSeconds(1)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(-40, -60, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(12, -60, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(1.3)
                .splineToSplineHeading(new Pose2d(backdropPos6.position.plus(new Vector2d(-6.5, 0)), Math.toRadians(0)), 0)
                .strafeToLinearHeading(backdropPos6.position.plus(new Vector2d(-20, 0)), 0)
                .splineToSplineHeading(new Pose2d(parkFarRed.plus(new Vector2d(-12, 0)), 0), 0)
                .splineToSplineHeading(new Pose2d(parkFarRed.plus(new Vector2d(0, 0)), 0), 0)
                .build()
        );

        blueBackBot1.runAction(blueBackBot1.getDrive().actionBuilder(blueBackBot1.getPose())
                .strafeToLinearHeading(new Vector2d(startBlueBack.position.x, startBlueBack.position.y - 5), startBlueBack.heading)
                .splineToSplineHeading(new Pose2d(backdropPos1.position.plus(new Vector2d(-7.5, 1.5)), Math.toRadians(-2)), 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(spikePosBlueBack.plus(new Vector2d(9.5, -2)), 0), -Math.PI)
                .waitSeconds(1)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(parkCloseBlue.plus(new Vector2d(-12, 0)), 0), 0)
                .splineToSplineHeading(new Pose2d(parkCloseBlue, 0), 0)
                .build()
        );

        blueBackBot2.runAction(blueBackBot2.getDrive().actionBuilder(blueBackBot2.getPose())
                .strafeToLinearHeading(new Vector2d(startBlueBack.position.x, startBlueBack.position.y - 5), startBlueBack.heading)
                .splineToSplineHeading(new Pose2d(backdropPos2.position.plus(new Vector2d(-7.5, 1.5)), Math.toRadians(-2)), 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(spikePosBlueBack.plus(new Vector2d(0, -7)), 0), -Math.PI)
                .waitSeconds(1)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(parkCloseBlue.plus(new Vector2d(-12, 0)), 0), 0)
                .splineToSplineHeading(new Pose2d(parkCloseBlue, 0), 0)
                .build()
        );

        blueBackBot3.runAction(blueBackBot3.getDrive().actionBuilder(blueBackBot3.getPose())
                .strafeToLinearHeading(new Vector2d(startBlueBack.position.x, startBlueBack.position.y - 5), startBlueBack.heading)
                .splineToSplineHeading(new Pose2d(backdropPos3.position.plus(new Vector2d(-7.5, 1.5)), Math.toRadians(-2)), 0)
                .waitSeconds(1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(spikePosBlueBack.plus(new Vector2d(-11.5, 4)), 0), -Math.PI)
                .waitSeconds(1)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(parkCloseBlue.plus(new Vector2d(-12, 0)), 0), 0)
                .splineToSplineHeading(new Pose2d(parkCloseBlue, 0), 0)
                .build()
        );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(blueBackBot1)
                .addEntity(blueBackBot2)
                .addEntity(blueBackBot3)
//                .addEntity(blueFrontBot)
//                .addEntity(redBackBot)
                .addEntity(redFrontBot1)
                .addEntity(redFrontBot2)
                .addEntity(redFrontBot3)
                .start();
    }
}