package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.util.Timing;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TagsTest")
//@Disabled

public class TagsTestOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    VisionSubsysytem vss;
    Timing.Timer readCamera;
    long readMilliSecs = 250;


    MecanumDriveSubsystemSimulation driveSim;


    @Override
    public void initialize() {
        vss = new VisionSubsysytem(this);

        driveSim = new MecanumDriveSubsystemSimulation(this);
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
            telemetryM.update(telemetry);
        }
        reset();
    }

    public double getAngleDegreesToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.getX() - robotPose.getX();
        double YDiff = targetPose.getY() - robotPose.getY();
        return Math.toDegrees(Math.atan2(YDiff, XDiff));
    }

    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.getX() - robotPose.getX();
        double YDiff = targetPose.getY() - robotPose.getY();
        return Math.sqrt(Math.pow(XDiff, 2) - Math.pow(YDiff, 2));
    }

    public void showTelemetry() {

        //  if (readCamera.done()) {

        double dist = 0;

        dist =getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());

        telemetryM.addData("Distance(from odometry)", f3(dist));
        List<LLResultTypes.FiducialResult> fr = vss.getFiducialResults();

        List<Integer> tagsSeen = vss.getTagsSeen(fr);
        telemetryM.addData("TagsSeen", tagsSeen);


        telemetryM.addData("ZoneFromTags", vss.findZoneFromTags(tagsSeen));
        telemetryM.addData("Current Zone", vss.getCurrentTagZone());
        telemetryM.addData("Is In Current Zone", vss.isInCurrentZone(fr));

        if (!fr.isEmpty()) {
            double[] temp = vss.getGroupTargetPose(fr);
            double x = temp[0];
            double y = temp[1];
            double z = temp[2];
            double yaw = temp[3];
            double pitch = temp[4];

            telemetryM.addData("X", f3M2I(x));
            telemetryM.addData("Y", f3M2I(y));
            telemetryM.addData("Z", f3M2I(z));
            telemetryM.addData("Yaw", f3(yaw));
            telemetryM.addData("Z", f3(pitch));


        }

        LLResult result = vss.getLLResults();
        if (result.isValid()) {

            telemetryM.addData("tx", f3(result.getTx()));
            telemetryM.addData("ty", f3(result.getTy()));
            telemetryM.addData("ta", f3(result.getTa()));
            telemetryM.addData("Tag Count", vss.getNumberTagsSeen(fr));

        }


    }

    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f3M2I(double v) {
        return String.format(Locale.US, "%.3f", v * 39.37);
    }

}


