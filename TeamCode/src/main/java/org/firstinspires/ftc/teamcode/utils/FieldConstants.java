package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

public class FieldConstants {

    public final static double fieldLength = 144;
    public final static double fieldWidth = 144;
    public final static double robotLength = 18;
    public final static double robotWidth = 18;


    public static double loadZoneYLength = 24;
    public static double loadZoneWidth = 12;
    public static double redLoadZoneYStart = 96;
    public static double blueLoadZoneYStart = 24;

    public final static double redHiveYCenter = 60;
    public final static double blueHiveYCenter = 84;


    public final static double redScoringHiveXCenter = 94;
    public final static double redAudienceHiveXCenter = 50;
    public final static double blueScoringHiveXCenter = 84;
    public final static double blueAudienceHiveXCenter = 50;


    //Start Poses
    public static Pose redScoringStartPose = new Pose(fieldLength - robotLength / 2, redHiveYCenter);
    public static Pose redAudienceStartPose = new Pose(robotLength / 2, redHiveYCenter);
    public static Pose blueScoringStartPose = new Pose(fieldLength - robotLength / 2, blueHiveYCenter);
    public static Pose blueAudienceStartPose = new Pose(robotLength / 2, blueHiveYCenter);


    //Park poses
    public static Pose redScoringParkPose = new Pose(loadZoneWidth,redLoadZoneYStart+loadZoneYLength);
    public static Pose redAudienceParkPose = new Pose(loadZoneWidth,redLoadZoneYStart);
    public static Pose blueScoringParkPose = new Pose(loadZoneWidth,blueLoadZoneYStart+loadZoneYLength);
    public static Pose blueAudienceParkPose = new Pose(loadZoneWidth,blueLoadZoneYStart);


    // Flower Poses
    public static Pose redAllianceWallFlowerPose = new Pose(0, 48);
    public static Pose redScoringWallFlowerPose = new Pose(48, fieldLength, Math.PI);

    public static Pose blueAllianceWallFlowerPose = new Pose(0, 96);
    public static Pose blueScoringWallFlowerPose = new Pose(48, fieldLength);


    //Hive targets
    ;

    public static Pose redAudienceHiveTargetPose = new Pose(redAudienceHiveXCenter, redHiveYCenter);
    public static Pose redScoringHiveTargetPose = new Pose(redScoringHiveXCenter, redHiveYCenter);
    //BLUE poses
    public static Pose blueAudienceHiveTargetPose = new Pose(blueAudienceHiveXCenter, blueHiveYCenter);
    public static Pose blueScoringHiveTargetPose = new Pose(blueScoringHiveXCenter, blueHiveYCenter);


}
