package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldConstants {

    public final static double fieldLength = 144;
    public final static double fieldWidth = 144;
    public final static double robotLength = 18;
    public final static double robotArmLengthAdder = 6;
    public final static double robotWidth = 18;

    public final static double flowerApproachClearance = 12;
    public static double loadZoneYLength = 24;
    public static double loadZoneXWidth = 12;
    public static double redLoadZoneYStart = 96;

    public final static double redHiveXCenter = 60;
    public final static double redScoringHiveYCenter = 86;
    public final static double redAudienceHiveYCenter = 54;


    //Start Poses
    public static Pose redScoringLoHiveStartPose =
            new Pose(redHiveXCenter,
                    fieldLength - robotLength / 2,
                    Math.PI / 2);
    public static Pose redAudienceHiHiveStartPose = new Pose(redHiveXCenter,
            robotLength / 2,
            -Math.PI / 2);


    //Park poses
    public static Pose redScoringParkPose =
            new Pose(loadZoneXWidth,
                    redLoadZoneYStart + loadZoneYLength,
                    Math.PI / 2);
    public static Pose redAudienceParkPose =
            new Pose(loadZoneXWidth,
                    redLoadZoneYStart,
                    Math.PI);


    public static Pose redAudienceParkPoseFromStart =
            new Pose(loadZoneXWidth,
                    redLoadZoneYStart,
                    -Math.PI / 2);


    // Flower Poses
    public static Pose redAllianceWallFlowerPose = new Pose(0, 48);
    public static Pose scoringWallFlowerPose = new Pose(48, fieldLength);


    //Hive targets

    public static Pose redAudienceHiveTargetPose = new Pose(redHiveXCenter, redAudienceHiveYCenter, -Math.PI / 2);
    public static Pose redScoringHiveTargetPose = new Pose(redHiveXCenter, redScoringHiveYCenter, Math.PI / 2);


    //Flower approach poses

    public static Pose scoringWallFlowerApproachPose =
            new Pose(scoringWallFlowerPose.getX(),
                    fieldLength - robotLength / 2 - robotArmLengthAdder - flowerApproachClearance,
                    Math.PI / 2);


    //Shoot poses

    public static Pose redScoringSecondShootPose =
            new Pose(scoringWallFlowerPose.getX(),
                    fieldLength - robotLength / 2 - robotArmLengthAdder,
                    Math.toRadians(80));
    public static Pose scoringWallFlowerApproachControlPointPose = new Pose(55, 114);


    //Control Points for path from alliance wall to scoring wall
    private final Pose point2Control1 = new Pose(28, 65, 0);
    private final Pose point2Control2 = new Pose(29.6127, 107.7505, 0);
    private final Pose point2Control3 = new Pose(60, 112, 0);

    public static final Pose redAllianceParkfromStartControlPointPose = new Pose(17, 35);
    public static final Pose lowHiveStartControlPointPose = new Pose(53, 108, 0);

    public static Pose redAllianceWallFlowerApproachPose =
            new Pose(redAllianceWallFlowerPose.getX() + robotArmLengthAdder + flowerApproachClearance,
                    redAllianceWallFlowerPose.getY(),
                    Math.PI);
    public static Pose redAllianceWallFlowerPickupPose =
            new Pose(redAllianceWallFlowerPose.getX() + robotLength / 2 + robotArmLengthAdder,
                    redAllianceWallFlowerPose.getY(),
                    Math.PI);


    // Flower pickup poses
    public static Pose scoringWallFlowerPickupPose =
            new Pose(scoringWallFlowerPose.getX(),
                    fieldLength - robotLength / 2 - robotArmLengthAdder,
                    Math.PI / 2);

    /**
     * BIOBUZZ: blue = red rotated 180° about field center (72,72).
     */
    static double FIELD = 144;

    public static Pose toBlue(Pose red) {
        return new Pose(
                FIELD - red.getX(),
                FIELD - red.getY(),
                AngleUnit.normalizeRadians(red.getHeading() + Math.PI));
    }

    public static Pose toBlueXY(Pose red) {
        return new Pose(
                FIELD - red.getX(),
                FIELD - red.getY());

    }

}
