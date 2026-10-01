package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.geometry.Pose;

public class FieldConstants {

    public final static double fieldLength = 144;
    public final static double fieldWidth = 144;
    public final static double robotLength = 18;
    public final static double robotWidth = 18;
    public final static double allianceLoadZoneYLength = 23;
    public final static double allianceLoadZoneXWidth = 11;
    public final static double allianceGardenZoneXLength = 23;
    public final static double allianceGardenZoneYWidth = 2;
    public final static double allianceGardenZoneYStart = 0;

    public final static double redHiveXCenter = 60;
    public final static double blueHiveXCenter = 84;


    public static double redLoadZoneYStart = 96;
    public static Pose redAudiencceStartPose = new Pose(redHiveXCenter, robotLength / 2);
    public static Pose redScoringStartPose = new Pose(redHiveXCenter, fieldLength - robotLength / 2);
    public static Pose redAllianceWallFlowerPose = new Pose(0, 48);
    public static Pose redScoringWallFlowerPose = new Pose(48, fieldLength);


}
