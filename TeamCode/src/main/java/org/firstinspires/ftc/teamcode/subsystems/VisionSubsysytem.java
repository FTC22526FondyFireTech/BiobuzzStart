package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class VisionSubsysytem extends SubsystemBase {

    private final Limelight3A limelight;

    private final int fiducialPipeline = 0;

    public int[] redScoringTags = new int[]{30, 31, 32, 33};//zone 0
    public int[] redAudienceTags = new int[]{34, 35, 36, 37};//zone 1
    public int[] blueAudienceTags = new int[]{38, 39, 40, 41};//zone 2
    public int[] blueScoringTags = new int[]{42, 43, 44, 45};//zone 3
    public int[] currentZoneTags = new int[]{0, 0, 0, 0};
    private final double hfov = 54.5;
    private final double vfov = 42;


    public static final double ppRedHiveCenterY = 60;
    public static final double ppBlueHiveCenterY = 84;

    public static final double scoringX = 90;
    public static final double audienceX = 64;

    public static final double shootHeight = 66;
    public Pose3D blueScoringShootTargetPose = new Pose3D(
            new Position(DistanceUnit.INCH, scoringX, ppBlueHiveCenterY, shootHeight, 0),
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    public Pose3D blueAudienceShootTargetPose = new Pose3D(
            new Position(DistanceUnit.INCH, audienceX, ppBlueHiveCenterY, shootHeight, 0),
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));

    public Pose3D redScoringShootTargetPose = new Pose3D(
            new Position(DistanceUnit.INCH, scoringX, ppRedHiveCenterY, shootHeight, 0),
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    public Pose3D redAudienceShootTargetPose = new Pose3D(
            new Position(DistanceUnit.INCH, audienceX, ppRedHiveCenterY, shootHeight, 0),
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));


    public int getCurrentTagZone() {
        return currentTagZone;
    }

    public void setCurrentTagZone(int currentZone) {
        currentTagZone = currentZone;
    }

    private int currentTagZone;


    public VisionSubsysytem(CommandOpMode opMode) {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        setCurrentZoneTags(0);
        opMode.telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(fiducialPipeline);


        // limelight.updateRobotOrientation(yaw); //MT2 if used

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public String getName() {
        return getStatus().getName();
    }

    public double getTemoerature() {
        return getStatus().getTemp();
    }

    public double getCpu() {
        return getStatus().getCpu();
    }

    public double getFPS() {
        return getStatus().getFps();
    }

    public double getPipelineNumber() {
        return getStatus().getPipelineIndex();
    }

    public String getPipelineType() {
        return getStatus().getPipelineType();
    }

    public LLResult getLLResults() {
        return limelight.getLatestResult();
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public double getPitch(LLResult result) {
        return result.getBotpose().getOrientation().getPitch();
    }

    public double getRoll(LLResult result) {
        return result.getBotpose().getOrientation().getRoll();
    }

    public double getYaw(LLResult result) {
        return result.getBotpose().getOrientation().getYaw();
    }

    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        LLResult result = getLLResults();
        if (result == null) {
            return Collections.emptyList();
        }
        return result.getFiducialResults();
    }

    public int getNumberTagsSeen(List<LLResultTypes.FiducialResult> llresults) {
        return llresults.size();
    }

    public List<Integer> getTagsSeen(List<LLResultTypes.FiducialResult> llresults) {
        List<Integer> ids = new ArrayList<>();
        int i = 0;
        for (LLResultTypes.FiducialResult fiducial : llresults) {
            ids.add(fiducial.getFiducialId());
            i++;
        }
        return ids;
    }

    public int findZoneFromTags(List<Integer> tagsSeen) {
        int[][] zoneTagSets = new int[][]{redScoringTags, redAudienceTags, blueAudienceTags, blueScoringTags};
        for (int zone = 0; zone < zoneTagSets.length; zone++) {
            int matches = 0;
            for (int zoneTag : zoneTagSets[zone]) {
                if (tagsSeen.contains(zoneTag)) {
                    matches++;
                }
            }
            if (matches >= 2) {
                return zone;
            }
        }
        return -1;
    }

    public int updateCurrentTagZone(List<LLResultTypes.FiducialResult> llresults) {
        int zone = findZoneFromTags(getTagsSeen(llresults));
        if (zone != -1) {
            setCurrentTagZone(zone);
        }
        return getCurrentTagZone();
    }

    public boolean isInCurrentZone(List<LLResultTypes.FiducialResult> llresults) {
        List<Integer> tagsSeen = getTagsSeen(llresults);
        int matches = 0;
        for (int currentZoneTag : currentZoneTags) {
            if (tagsSeen.contains(currentZoneTag)) {
                matches++;
            }
        }
        return matches >= 2;
    }

    public double getAvgDistance(LLResult result) {
        return result.getBotposeAvgDist();
    }

    public double getAvgArea(LLResult result) {
        return result.getBotposeAvgArea();
    }


    public void setPipeline(int n) {
        limelight.pipelineSwitch(n);
    }

    public void setCurrentZoneTags(int zone) {
        switch (zone) {

            case 0:
                currentZoneTags = Arrays.copyOf(redScoringTags, 4);
                break;
            case 1:
                currentZoneTags = Arrays.copyOf(redAudienceTags, 4);
                break;
            case 2:
                currentZoneTags = Arrays.copyOf(blueAudienceTags, 4);
                break;
            case 3:
                currentZoneTags = Arrays.copyOf(blueScoringTags, 4);
                break;
            default:
                break;
        }
    }

    public Pose3D getCurrentZoneShootTarget(int zone) {
        switch (zone) {
            case 0:
                return redScoringShootTargetPose;
            case 1:
                return redAudienceShootTargetPose;
            case 2:
                return blueAudienceShootTargetPose;
            case 3:
                return blueScoringShootTargetPose;
            default:
                return new Pose3D(new Position(DistanceUnit.INCH, audienceX, ppBlueHiveCenterY, shootHeight, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));

        }
    }


    @Override
    public void periodic() {

    }
}


