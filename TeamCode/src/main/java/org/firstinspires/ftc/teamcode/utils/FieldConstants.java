package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

public class FieldConstants {

    public final static double fieldLength = 144;
    public final static double fieldWidth = 144;
    public final static double robotLength = 18;
    public final static double robotArmLengthAdder = 6;
    public final static double robotWidth = 18;
    public final static double flowerExtensionCenterFromWall = 2.5;

    public final static double flowerApproachClearance = 6;
    public static double loadZoneYLength = 24;
    public static double loadZoneXWidth = 12;
    public static double redLoadZoneYStart = 96;
    public static double blueLoadZoneYStart = 24;

    public final static double redHiveXCenter = 60;
    public final static double blueHiveXCenter = 84;


    public final static double redScoringHiveYCenter = 94;
    public final static double redAudienceHiveYCenter = 50;
    public final static double blueScoringHiveYCenter = 84;
    public final static double blueAudienceHiveYCenter = 50;


    //Start Poses
    public static Pose redScoringLoHiveStartPose = new Pose(redHiveXCenter, fieldLength - robotLength / 2, -Math.PI / 2);
    public static Pose redAudienceStartPose = new Pose(redHiveXCenter, robotLength / 2, Math.PI / 2);
    public static Pose blueScoringStartPose = new Pose(blueHiveXCenter, fieldLength - robotLength / 2, Math.PI / 2);
    public static Pose blueAudienceLoHiveStartPose = new Pose(blueHiveXCenter, robotLength / 2, Math.PI / 2);

//Shoot poses

    public static Pose scoringSecondShootPose = new Pose(redHiveXCenter, redAudienceHiveYCenter);
    public static Pose audienceSecondShootPose = new Pose(redHiveXCenter, redAudienceHiveYCenter);


    //Park poses
    public static Pose redScoringParkPose = new Pose(loadZoneXWidth, redLoadZoneYStart + loadZoneYLength, -Math.PI / 2);
    public static Pose redAudienceParkPose = new Pose(loadZoneXWidth, redLoadZoneYStart);
    public static Pose blueScoringParkPose = new Pose(loadZoneXWidth, blueLoadZoneYStart);
    public static Pose blueAudienceParkPose = new Pose(fieldWidth - loadZoneXWidth, blueLoadZoneYStart, Math.PI / 2);


    // Flower Poses
    public static Pose redAllianceWallFlowerPose = new Pose(0, 48);
    public static Pose scoringWallFlowerPose = new Pose(48, fieldLength);

    public static Pose blueAllianceWallFlowerPose = new Pose(fieldWidth, 96);
    public static Pose audienceWallFlowerPose = new Pose(96, 0);


    //Hive targets

    public static Pose redAudienceHiveTargetPose = new Pose(redHiveXCenter, redAudienceHiveYCenter);
    public static Pose redScoringHiveTargetPose = new Pose(redHiveXCenter, redScoringHiveYCenter);
    //BLUE poses
    public static Pose blueAudienceHiveTargetPose = new Pose(blueHiveXCenter, blueAudienceHiveYCenter);
    public static Pose blueScoringHiveTargetPose = new Pose(blueHiveXCenter, blueScoringHiveYCenter);


    //Flower approach poses

    public static Pose scoringWallFlowerApproachPose =
            new Pose(scoringWallFlowerPose.getX(), fieldLength - flowerExtensionCenterFromWall - flowerApproachClearance - robotLength / 2 - robotArmLengthAdder, -Math.PI / 2);
    public static Pose scoringWallFlowerApproachControlPointPose = new Pose(55,91);

    public static Pose audienceWallFlowerApproachPose =
            new Pose(audienceWallFlowerPose.getX(), flowerExtensionCenterFromWall + robotLength / 2 - +robotArmLengthAdder, -Math.PI);


    // Flower pickup poses
    public static Pose

            scoringWallFlowerPickupPose = new Pose(scoringWallFlowerPose.getX(), +fieldLength - flowerExtensionCenterFromWall / 2 - robotLength / 2, -Math.PI / 2);
    public static Pose audienceWallFlowerPickupPose = new Pose(audienceWallFlowerPose.getX(), flowerExtensionCenterFromWall + robotLength / 2, -Math.PI);

//April TagPoses

    public static double tag30X = redHiveXCenter + 6.5;
    public static Pose tag30 = new Pose(tag30X, redScoringHiveYCenter, Math.toRadians(39));


}
