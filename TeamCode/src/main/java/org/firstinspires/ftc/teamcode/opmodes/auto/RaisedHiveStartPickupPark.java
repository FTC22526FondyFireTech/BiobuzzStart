package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.AlignToPoseCommand;
import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.commands.AlignToPoseCommandSim;
import org.firstinspires.ftc.teamcode.simulator.commands.AutoCommandsSim;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.AutoCommands;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GetRPMFromDistance;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.Arrays;

/**
 * {@link MecanumDriveSubsystemSimulation} instead of real hardware - no robot required.
 * Swap {@link Constants#createFollower(com.qualcomm.robotcore.hardware.HardwareMap)}
 * for {@link SimulatorConstants#createSimulatedFollower(MecanumDriveSubsystemSimulation)} and
 * everything else (PathChains, FollowPathCommand, follower.update()/getPose()) works unchanged,
 * since both factories hand back a real {@code Follower}.
 */

/**
 * Used only when partner can score in the initial raised hive.
 * Robot starts at the red or blue low hive and waits until it raises before shooting.
 * Then moves to pick up from adjacent flower and shoots those before parking.
 * If hive doesn't raise before 20 seconds robot goes straight to park.
 *
 *
 *
 *
 */
@Autonomous(name = "Raised Hive Start Pickup Park", group = "Auto")
public class RaisedHiveStartPickupPark extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private MecanumDriveSubsystemSimulation driveSim;
    private IntakeSubsystem intake;
    private PollenShooterSubsystem pss;
    private NectarShooterSubsystem nss;
    private Follower follower;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Pose startPose, flowerApproachPose, controlPointPose, flowerPickupPose, parkPose, parkPoseFromStart;
    private PathChain approachFlower, pickupFlower, retractFlower, returnToStart, park, parkFromStart;

    private boolean isBlueAlliance;

    private ElapsedTime autoTimer;
    private AutoCommandsSim acSim;

    private AutoCommands ac;
    private GetRPMFromDistance grpmpollenfd;
    private int seqStep;
    private double distancetoTarget;
    private double targetRPMFromDistance;

    public boolean hiveHasFallen = true;


    @Override
    public void initialize() {
        super.reset();
        isBlueAlliance = GlobalData.isBlueAlliance();

        if (!Configurables.doSimulation) {
            drive = new MecanumDriveSubsystem(this.hardwareMap);
        } else {
            driveSim = new MecanumDriveSubsystemSimulation(this);
        }

        intake = new IntakeSubsystem(this.hardwareMap);
        pss = new PollenShooterSubsystem(this.hardwareMap);

        nss = new NectarShooterSubsystem(this.hardwareMap);

//if (!Configurables.doSimulation) {
        ac = new AutoCommands(drive, pss, nss);

        //
        acSim = new AutoCommandsSim(driveSim, pss, nss);
        //  }

        grpmpollenfd = new GetRPMFromDistance(driveSim, pss);

        GlobalData.autoHasRun = true;

        autoTimer = new ElapsedTime();
        startPose = forAlliance(FieldConstants.redAudienceHiHiveStartPose, isBlueAlliance);
        flowerApproachPose = forAlliance(FieldConstants.redAllianceWallFlowerApproachPose, isBlueAlliance);
        flowerPickupPose = forAlliance(FieldConstants.redAllianceWallFlowerPickupPose, isBlueAlliance);
        parkPose = forAlliance(FieldConstants.redAudienceParkPose, isBlueAlliance);
        parkPoseFromStart = forAlliance(FieldConstants.redAudienceParkPoseFromStart, isBlueAlliance);

        controlPointPose = forAlliance(FieldConstants.redAllianceParkfromStartControlPointPose, isBlueAlliance);


        // Only this line differs from PedroAutoSample.initialize() - everything below is
        // identical Follower/PathChain/FollowPathCommand usage.

        if (!Configurables.doSimulation)
            follower = Constants.createFollower(this.hardwareMap);
        else
            follower = SimulatorConstants.createSimulatedFollower(driveSim);

        follower.setStartingPose(startPose);

        buildPaths();


        hiveHasFallen = false;

        schedule(

                ShootPickupPark()
        );

    }


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

    public static Pose forAlliance(Pose redPose, boolean isBlue) {
        return isBlue ? toBlue(redPose) : redPose;
    }


    private Command ShootPickupPark() {
        return
                Commands.sequence(

                        Commands.sequence(
                                new WaitCommand(500),
                                checkSequenceStepCommand(),
                                Commands.defer(() -> pss.setTargetRPMCommand(targetRPMFromDistance), Arrays.asList()),
                                new ConditionalCommand(
                                        ac.autoShootCommand(),
                                        acSim.autoShootPollenCommand(),
                                        () -> !Configurables.doSimulation),
                                Commands.parallel(
                                        checkSequenceStepCommand(),
                                        new FollowPathCommand(follower, approachFlower, true),
                                        intake.runIntakeCommand(),
                                        intake.armDownCommand()),
                                checkSequenceStepCommand(),
                                new WaitCommand(500),
                                new FollowPathCommand(follower, pickupFlower, true).setGlobalMaxPower(0.25),
                                new WaitCommand(1500),
                                checkSequenceStepCommand(),
                                new FollowPathCommand(follower, retractFlower, true),
                                Commands.waitMillis(1500),
                                Commands.parallel(
                                        intake.stopIntakeCommand(),
                                        intake.armUpCommand(),
                                        new WaitCommand(500)),
                                checkSequenceStepCommand(),
                                Commands.defer(() ->
                                                new ConditionalCommand(
                                                        new FollowPathCommand(follower, park, true),
                                                        Commands.sequence(
                                                                new FollowPathCommand(follower, returnToStart, true),
                                                                checkSequenceStepCommand(),

                                                                new AlignToPoseCommandSim(driveSim),
                                                                //    new AlignToPoseCommand(drive),

                                                                checkSequenceStepCommand(),
                                                                Commands.waitMillis(500),
                                                                new ConditionalCommand(
                                                                        ac.autoShootCommand(),
                                                                        acSim.autoShootPollenCommand(),
                                                                        () -> !Configurables.doSimulation),
                                                                checkSequenceStepCommand(),
                                                                Commands.waitMillis(500),
                                                                new FollowPathCommand(follower, parkFromStart, true),
                                                                checkSequenceStepCommand(),
                                                                new WaitCommand(250)),

                                                        () -> hiveHasFallen),
                                        Arrays.asList())));
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        autoTimer.reset();

        while (!isStopRequested() && opModeIsActive() && GlobalData.allianceIsConfirmed && autoTimer.seconds() <= 30) {
            run();

            follower.update();

            distancetoTarget = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());

            targetRPMFromDistance = grpmpollenfd.getPollenRPMFromDistanceSim(distancetoTarget);

            telemetryM.addData("Auto Timer", autoTimer.seconds());
            telemetryM.addData("Follower Is Busy", follower.isBusy());
            telemetryM.addData("Shooting", pss.shooterServoTargetPosition == PollenShooterSubsystem.shootPosition);
            telemetryM.addData("Intake Running", intake.intakeRunning);
            telemetryM.addData("Intake Arm Down", intake.getArmPosition() == IntakeSubsystem.downPosition);

            telemetryM.addData("SeqStep", seqStep);

            telemetryM.update(telemetry);
        }
        reset();
    }

    public void buildPaths() {

        approachFlower = buildBezierLine(startPose, flowerApproachPose);
        pickupFlower = buildBezierLine(flowerApproachPose, flowerPickupPose);
        retractFlower = buildBezierLine(flowerPickupPose, flowerApproachPose);
        returnToStart = buildBezierLine(flowerApproachPose, startPose);
        park = buildBezierLine(flowerPickupPose, parkPose);
        parkFromStart = buildBezierCurve(startPose, controlPointPose, parkPoseFromStart);

    }

    PathChain buildBezierLine(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                //.setConstantHeadingInterpolation(end.getHeading())
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    PathChain buildBezierCurve(Pose start, Pose control, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                // .setConstantHeadingInterpolation(end.getHeading())
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    public double getDistanceToTarget(Pose targetPose, Pose robotPose) {
        double XDiff = targetPose.getX() - robotPose.getX();
        double YDiff = targetPose.getY() - robotPose.getY();
        return Math.hypot(XDiff, YDiff);
    }

    public Command checkSequenceStepCommand() {
        return Commands.runOnce(() -> seqStep++);
    }


}
