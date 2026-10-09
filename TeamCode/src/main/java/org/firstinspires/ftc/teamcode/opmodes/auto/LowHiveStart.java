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

import org.firstinspires.ftc.teamcode.commands.CheckForOdometryZoneTags;
import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

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

    private VisionSubsysytem vss;
    private Follower follower;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private Pose startPose, secondShootPose, flowerApproachPose, flowerPickupPose, parkPose;
    private PathChain approachFlower, pickupFlower, score, park;


    boolean allianceSelected;
    boolean choicesComplete;
    private boolean allianceIsSelected;


    @Override
    public void initialize() {
        super.reset();

        if (!Configurables.doSimulation)
            drive = new MecanumDriveSubsystem(this.hardwareMap);
        else
            driveSim = new MecanumDriveSubsystemSimulation(this);
        vss = new VisionSubsysytem(this);

        GlobalData.selectAlliance(this);


        if (GlobalData.isRedAlliance()) {
            startPose = FieldConstants.redScoringStartPose;
            flowerApproachPose = FieldConstants.scoringWallFlowerApproachPose;
            flowerPickupPose = FieldConstants.scoringWallFlowerPickupPose;
            secondShootPose = FieldConstants.scoringSecondShootPose;
            parkPose = FieldConstants.redScoringParkPose;
        } else {
            startPose = FieldConstants.blueAudienceStartPose;
            flowerApproachPose = FieldConstants.audienceWallFlowerApproachPose;
            flowerPickupPose = FieldConstants.audienceWallFlowerPickupPose;
            secondShootPose = FieldConstants.audienceSecondShootPose;
            parkPose = FieldConstants.blueAudienceParkPose;
        }

        // Only this line differs from PedroAutoSample.initialize() - everything below is
        // identical Follower/PathChain/FollowPathCommand usage.

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

    private Command ShootPickupPark() {
        return Commands.sequence(
                new CheckForOdometryZoneTags(vss).withTimeout(20000),

                new ConditionalCommand(
                        Commands.sequence(
                                new WaitCommand(500),
                                new FollowPathCommand(follower, pickupFlower).setGlobalMaxPower(0.5),
                                new FollowPathCommand(follower, score),
                                new FollowPathCommand(follower, park, false)),

                        new FollowPathCommand(follower, park, false),

                        () -> vss.isInOdometryZone()));


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive() && GlobalData.allianceIsConfirmed) {
            run();

            follower.update();

            telemetryM.addData("X", follower.getPose().getX());
            telemetryM.addData("Y", follower.getPose().getY());
            telemetryM.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetryM.addData("Busy", follower.isBusy());

            telemetryM.update(telemetry);
        }
        reset();
    }

    public Pose flipBlueToRedPose(Pose blue) {
        double x = blue.getX();
        double y = blue.getY();
        x = SimulatorConstants.width - x;
        double h = blue.getHeading();
        return new Pose(x, y, Math.PI - h);
    }

    public void buildPaths() {
        approachFlower = buildBezierLine(startPose, flowerApproachPose);

        pickupFlower = buildBezierLine(flowerApproachPose, flowerPickupPose);
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


}
