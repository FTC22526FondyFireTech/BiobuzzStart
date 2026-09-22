package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.geometry.Pose2d;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TagsTest")
//@Disabled

public class TagsTestOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    VisionSubsysytem vss;
    Timing.Timer readCamera;
    long readMilliSecs = 250;


    @Override
    public void initialize() {
        vss = new VisionSubsysytem(this);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);
        readCamera = new Timing.Timer(readMilliSecs, TimeUnit.MILLISECONDS);
        readCamera.start();

    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();
            showTelemetry();
            //     telemetry.addData("Array",testPose);
            telemetryM.update(telemetry);
        }
        reset();
    }

    public double getAngleDegreesToTarget(Pose2d targetPose, Pose2d robotPose) {
        double XDiff = targetPose.getX() - robotPose.getX();
        double YDiff = targetPose.getY() - robotPose.getY();
        return Math.toDegrees(Math.atan2(YDiff, XDiff));
    }

    public double getDistanceToTarget(Pose2d targetPose, Pose2d robotPose) {
        double XDiff = targetPose.getX() - robotPose.getX();
        double YDiff = targetPose.getY() - robotPose.getY();
        return Math.sqrt(Math.pow(XDiff, 2) - Math.pow(YDiff, 2));
    }

    public void showTelemetry() {
        if (readCamera.done()) {


            List<LLResultTypes.FiducialResult> fr = vss.getFiducialResults();

            List<Integer> tagsSeen = vss.getTagsSeen(fr);
            telemetryM.addData("TagsSeen", tagsSeen);

            telemetryM.addData("ZoneFromTags", vss.findZoneFromTags(tagsSeen));
            telemetryM.addData("Current Zone", vss.getCurrentTagZone());
            telemetryM.addData("Is In Current Zone", vss.isInCurrentZone(fr));

            LLResult result = vss.getLatestResult();
            if (result.isValid()) {
                telemetryM.addData("AvgDistance", vss.getAvgDistance(result));
                telemetryM.addData("AvgArea", vss.getAvgArea(result));

                telemetryM.addData("Yaw", vss.getYaw(result));
                telemetryM.addData("Roll", vss.getRoll(result));

                telemetryM.addData("tx", result.getTx());
                telemetryM.addData("ty", result.getTy());
                telemetryM.addData("ta", result.getTa());
                telemetryM.addData("Tag Count", result.getBotposeTagCount());
                telemetryM.addData("Pitch", vss.getPitch(result));
                telemetryM.addData("Yaw", vss.getYaw(result));
                telemetryM.addData("Roll", vss.getRoll(result));

            }

        }

    }

}