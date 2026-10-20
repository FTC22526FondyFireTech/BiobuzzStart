package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.AimAtLLTXCommand;
import org.firstinspires.ftc.teamcode.commands.AimAtPoseCommand;
import org.firstinspires.ftc.teamcode.commands.AlignToPoseCommand;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.commands.DriveToPoseCommand;
import org.firstinspires.ftc.teamcode.simulator.commands.AlignToPoseCommandSim;
import org.firstinspires.ftc.teamcode.simulator.commands.DriveSimCommand;
import org.firstinspires.ftc.teamcode.simulator.commands.DriveToPoseCommandSim;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GPTrig;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.Arrays;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Teleop")
//@Disabled

public class TeleopOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;

    GPTrig leftTrigger;
    GPTrig rightTrigger;
    MecanumDriveSubsystem drive;
    VisionSubsysytem vss;
    IntakeSubsystem intake;

    PollenShooterSubsystem pss;

    NectarShooterSubsystem nss;

    AutoCommands ac;

    private Follower follower;

    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f3M2I(double v) {
        return String.format(Locale.US, "%.3f", v * 39.37);
    }


    /**
     * Teleop Buttons
     * <p>
     * Start Intake
     * Stop Intake
     * Lower Arm Hold/Release?
     * Raise Arm
     * <p>
     * Start Pollen Shooter Combine with Nectar start.
     * Start Nectar Shooter
     * Stop Pollen Shooter Combine with Nectar stop.
     * Stop Nectar Shooter
     * <p>
     * Lock on Target Hold
     * <p>
     * Preset Pose (if Odometry off)
     * Toggle Alliance (if wrong somehow)
     *
     *
     */


    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);

        if (!Configurables.doSimulation) {
            if (GlobalData.autoHasRun) {
                drive.setPose(GlobalData.robotPose);
                GlobalData.autoHasRun = false;
            } else
                drive.setPose(FieldConstants.redAudienceParkPose);
        }

        if (!GlobalData.isBlueAlliance() && !GlobalData.isRedAlliance())
            GlobalData.setRedAlliance();

        driverGamepad = new GamepadEx(gamepad1);

        drive = new MecanumDriveSubsystem(this.hardwareMap, new Pose());
        drive.setDefaultCommand(new DriveCommand(drive,
                () -> driverGamepad.getLeftY(),
                () -> -driverGamepad.getLeftX(),
                () -> driverGamepad.getRightX()));


        vss = new VisionSubsysytem(this);

        pss = new PollenShooterSubsystem(this.hardwareMap);

        nss = new NectarShooterSubsystem(this.hardwareMap);

        intake = new IntakeSubsystem(this.hardwareMap);

        ac = new AutoCommands(drive, pss, nss);


        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new AlignToPoseCommand(drive, () -> driverGamepad.getLeftY(), () -> driverGamepad.getLeftX()));


        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        Commands.defer(() ->
                                new DriveToPoseCommand(drive, GlobalData.getActiveStartPose()), Arrays.asList()));


        follower = Constants.createFollower(this.hardwareMap);

        driverGamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whileActiveOnce(GlobalData.toggleAllianceCommand());


        driverGamepad.getGamepadButton(GamepadKeys.Button.START)
                .whileActiveOnce(drive.setPoseCommand(new Pose()));


        //Triggers

        leftTrigger = new GPTrig(driverGamepad, GPTrig.Side.LEFT);
        rightTrigger = new GPTrig(driverGamepad, GPTrig.Side.RIGHT);

        leftTrigger.whenActive(intake.runIntakeCommand())
                .whenInactive(intake.stopIntakeCommand())
                .whenInactive(intake.armUpCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(intake.armDownCommand());

        rightTrigger.whenActive(new AimAtPoseCommand(drive, () -> driverGamepad.getLeftY(), () -> driverGamepad.getLeftX()));

//        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenHeld(ac.pollenShootUsingDistanceCommand());


        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whileActiveOnce(drive.setPoseCommand(new Pose(12, 12, 0)));

        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new AimAtLLTXCommand(drive, vss, () -> -driverGamepad.getLeftY(), () -> driverGamepad.getLeftX()));

        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new AimAtPoseCommand(drive, () -> driverGamepad.getLeftY(),
                        () -> driverGamepad.getLeftX()));


        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenHeld(intake.runIntakeCommand())
                .whenReleased(intake.stopIntakeCommand());


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();


            drive.setRobotCentric(driverGamepad.getGamepadButton(
                    GamepadKeys.Button.RIGHT_BUMPER).get());
            drive.showTelemetry(telemetryM);


            //intake.showTelemetry(telemetryM);
            telemetryM.addData("IsBusy", follower.isBusy());
            //telemetryM.addData("RobotCentric", driveSim.isRobotCentric());

            double dist = 0;
            double angle = 0;
            dist = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), drive.getPose());
            angle = getAngleToTarget(GlobalData.getActiveHiveTargetPose(), drive.getPose());


            telemetryM.addData("DistFromOdometry", f3(dist));
            telemetryM.addData("AngleFromOdometry", f3(angle));

            telemetryM.addData("TX", f3(GlobalData.getActiveHiveTargetPose().getX()));
            telemetryM.addData("TY", f3(GlobalData.getActiveHiveTargetPose().getY()));

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


            telemetryM.update(telemetry);
        }
        reset();
    }


    public double getAngleToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.getX() - robotPose.getX();
        double YDiff = targetPose.getY() - robotPose.getY();
        return Math.toDegrees(Math.atan2(YDiff, XDiff));
    }

    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.getX() - robotPose.getX();
        double YDiff = targetPose.getY() - robotPose.getY();
        telemetryM.addData("XDIFF", f3(XDiff));
        telemetryM.addData("YDIFF", f3(YDiff));

        // return Math.sqrt(Math.pow(XDiff, 2) + Math.pow(YDiff, 2));
        return Math.hypot(XDiff, YDiff);
    }


}