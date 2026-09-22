package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

/**
 * {@link MecanumDriveSubsystemSimulation} instead of real hardware - no robot required.
 * Swap {@link Constants#createFollower(com.qualcomm.robotcore.hardware.HardwareMap)}
 * for {@link SimulatorConstants#createSimulatedFollower(MecanumDriveSubsystemSimulation)} and
 * everything else (PathChains, FollowPathCommand, follower.update()/getPose()) works unchanged,
 * since both factories hand back a real {@code Follower}.
 */
@Autonomous(name = "Pedro Auto Samples", group = "Auto")
public class PedroAutoSamples extends CommandOpMode {

    private MecanumDriveSubsystem drive;
    private MecanumDriveSubsystemSimulation driveSim;
    private Follower follower;
    TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    private final Pose blueStartPose = new Pose(9, 111, Math.toRadians(-90));
    private final Pose blueScorePose = new Pose(16, 128, Math.toRadians(135));
    private final Pose bluePickup1Pose = new Pose(30, 121, Math.toRadians(0));
    private final Pose blueParkPose = new Pose(68, 96, Math.toRadians(-90));
    private Pose startPose, scorePose, pickup1Pose, parkPose;
    private PathChain scorePreload, grabPickup1, scorePickup1, park;


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


        GlobalData.selectAlliance(this);


        if (GlobalData.isRedAlliance()) {
            startPose = flipBlueToRedPose(blueStartPose);
            scorePose = flipBlueToRedPose(blueScorePose);
            pickup1Pose = flipBlueToRedPose(bluePickup1Pose);
            parkPose = flipBlueToRedPose(blueParkPose);
        } else {
            startPose = blueStartPose;
            scorePose = blueScorePose;
            pickup1Pose = bluePickup1Pose;
            parkPose = blueParkPose;

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

                Commands.sequence(
                        new FollowPathCommand(follower, scorePreload),
                        new WaitCommand(500),

                        new FollowPathCommand(follower, grabPickup1).setGlobalMaxPower(0.5),
                        new FollowPathCommand(follower, scorePickup1),

                        new FollowPathCommand(follower, park, false))
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive() &&GlobalData.allianceIsConfirmed) {
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
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(68, 110), // Control point
                        parkPose)
                )
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }


}
