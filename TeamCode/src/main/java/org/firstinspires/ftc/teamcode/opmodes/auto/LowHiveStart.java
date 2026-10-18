package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.CheckForOdometryZoneTags;
import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.commands.AlignToPoseCommandSim;
import org.firstinspires.ftc.teamcode.simulator.commands.AutoCommandsSim;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.NectarShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GetRPMFromDistance;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.Arrays;
import java.util.Locale;

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
@Autonomous(name = "Low Hive Start", group = "Auto")
public class LowHiveStart extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private MecanumDriveSubsystemSimulation driveSim;
    private PollenShooterSubsystem pss;
    private NectarShooterSubsystem nss;
    private IntakeSubsystem intake;

    private VisionSubsysytem vss;
    private Follower follower;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Pose startPose, controlPointPose, flowerApproachPose, flowerPickupPose, secondShootPose, parkPose;
    private PathChain approachFlower, pickupFlower, score, park;
    private AutoCommandsSim acSim;

    private GetRPMFromDistance grpmpollenfd;

    boolean allianceSelected;
    boolean choicesComplete;
    private boolean allianceIsSelected;

    private int seqStep;
    private double distancetoTarget;
    private double targetRPMFromDistance;

    static String f3(double v) {
        return String.format(Locale.US, "%.3f", v);
    }

    static String f1(double v) {
        return String.format(Locale.US, "%.1f", v);
    }

    static String f0(double v) {
        return String.format(Locale.US, "%.0f", v);
    }

    static String f3M2I(double v) {
        return String.format(Locale.US, "%.3f", v * 39.37);
    }


    @Override
    public void initialize() {
        super.reset();

        if (!Configurables.doSimulation)
            drive = new MecanumDriveSubsystem(this.hardwareMap);
        else
            driveSim = new MecanumDriveSubsystemSimulation(this);

        vss = new VisionSubsysytem(this);

        pss = new PollenShooterSubsystem(this.hardwareMap);

        intake = new IntakeSubsystem(this.hardwareMap);

        acSim = new AutoCommandsSim(driveSim, pss, nss);

        grpmpollenfd = new GetRPMFromDistance(driveSim, pss);

        //   GlobalData.selectAlliance(this);

        GlobalData.autoHasRun = true;


        startPose = forAlliance(FieldConstants.redScoringLoHiveStartPose, GlobalData.isBlueAlliance());
        flowerApproachPose = forAlliance(FieldConstants.scoringWallFlowerApproachPose, GlobalData.isBlueAlliance());
        flowerPickupPose = forAlliance(FieldConstants.scoringWallFlowerPickupPose, GlobalData.isBlueAlliance());
        secondShootPose = forAlliance(FieldConstants.scoringWallFlowerApproachPose, GlobalData.isBlueAlliance());
        parkPose = forAlliance(FieldConstants.redScoringParkPose, GlobalData.isBlueAlliance());


        controlPointPose = GlobalData.isBlueAlliance()
                ? FieldConstants.toBlueXY(FieldConstants.scoringWallFlowerApproachControlPointPose)
                : FieldConstants.scoringWallFlowerApproachControlPointPose;


        if (!Configurables.doSimulation)
            follower = Constants.createFollower(this.hardwareMap);
        else
            follower = SimulatorConstants.createSimulatedFollower(driveSim);

        follower.setStartingPose(startPose);

        buildPaths();

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
                        new CheckForOdometryZoneTags(vss).withTimeout(20000),
                        new ConditionalCommand(
                                Commands.sequence(
                                        checkSequenceStepCommand(),
                                        //  Commands.waitUntil(() -> targetRPMFromDistance > 500),
                                        checkSequenceStepCommand(),
                                        Commands.defer(() -> pss.setTargetRPMCommand(targetRPMFromDistance), Arrays.asList()),
                                        acSim.autoShootPollenCommand(),
                                        Commands.parallel(
                                                checkSequenceStepCommand(),
                                                new FollowPathCommand(follower, approachFlower),
                                                intake.runIntakeCommand(),
                                                intake.armDownCommand()),
                                        checkSequenceStepCommand(),
                                        new WaitCommand(500),
                                        new FollowPathCommand(follower, pickupFlower).setGlobalMaxPower(0.25),
                                        new WaitCommand(1500),
                                        checkSequenceStepCommand(),
                                        new FollowPathCommand(follower, score, false),
                                        checkSequenceStepCommand(),
                                        Commands.parallel(
                                                intake.stopIntakeCommand(),
                                                intake.armUpCommand(),
                                                new AlignToPoseCommandSim(driveSim)),

                                        Commands.defer(() -> pss.setTargetRPMCommand(targetRPMFromDistance), Arrays.asList()),

                                        acSim.autoShootPollenCommand(),
                                        new WaitCommand(500),
                                        new FollowPathCommand(follower, park, false),
                                        new WaitCommand(250)),

                                new FollowPathCommand(follower, park, false),

                                () -> vss.isInOdometryZone() || Configurables.doSimulation));


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();
        while (!isStopRequested() && opModeIsActive() && GlobalData.allianceIsConfirmed) {

            if (GlobalData.isRedAlliance() || GlobalData.isBlueAlliance()) {
                run();

                follower.update();

                distancetoTarget = getDistanceToTarget(GlobalData.getActiveHiveTargetPose(), driveSim.getOdometry().getRobotPose());

                targetRPMFromDistance = grpmpollenfd.getPollenRPMFromDistanceSim(distancetoTarget);

                //   pss.setTargetRPM(targetRPMFromDistance);

                telemetryM.addData("SEQSTEP", seqStep);

                telemetryM.addData("Dist To Tgt", f0(distancetoTarget));
                telemetryM.addData("TGTRPMFD", f0(targetRPMFromDistance));
                telemetryM.addData("IntakeRPM", f0(intake.getVelocityRPM()));
                telemetryM.addData("TGET RPM", f0(pss.getTargetRPM()));
                telemetryM.addData("ACT RPM", f0(pss.getPollenShooterRPM()));

                telemetryM.addData("Intake Arm Position", intake.getArmPosition());
                telemetryM.addData("PSSRVTGT", pss.shooterServoTargetPosition);
                telemetryM.addData("PSSATSPEED", pss.pollenShooterAtSpeed());

//                telemetryM.addData("X", follower.getPose().getX());
//                telemetryM.addData("Y", follower.getPose().getY());
//                telemetryM.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
//                telemetryM.addData("Busy", follower.isBusy());

                telemetryM.update(telemetry);
            }
        }


        reset();
    }

    public void buildPaths() {

        approachFlower = buildBezierCurve(startPose, controlPointPose, flowerApproachPose);
        pickupFlower = buildBezierLine(flowerApproachPose, flowerPickupPose);
        score = buildBezierLine(flowerPickupPose, secondShootPose);
        park = buildBezierLine(secondShootPose, parkPose);

    }

    PathChain buildBezierLine(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    PathChain buildBezierCurve(Pose start, Pose control, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }


    PathChain buildBezierCurve(Pose start, Pose control, Pose control1, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, control1, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    PathChain buildBezierCurve(Pose start, Pose control, Pose control1, Pose control2, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(start, control, control1, control2, end))
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
