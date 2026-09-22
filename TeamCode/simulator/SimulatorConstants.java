package org.firstinspires.ftc.teamcode.simulator;

import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.SimulatedMecanumDrivetrainForFollower;
import org.firstinspires.ftc.teamcode.simulator.simulators.SimulatedLocalizer;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * Builds a Follower that drives {@link MecanumDriveSubsystemSimulation} instead of real
 * hardware, for running the same PathChains/autonomous code against the simulator.
 * {@code Follower} itself has no hardware dependency - it only needs a {@code Localizer}
 * and a {@code Drivetrain} - so this bypasses {@code FollowerBuilder} (which requires a
 * {@code HardwareMap}) and constructs the real {@link Follower} directly, reusing
 * {@link Constants#followerConstants}/{@link Constants#driveConstants}/
 * {@link Constants#pathConstraints} so simulated runs are tuned the same as the real robot.
 */
public class SimulatorConstants {

    public static Follower createSimulatedFollower(MecanumDriveSubsystemSimulation driveSim) {
        SimulatedLocalizer localizer = new SimulatedLocalizer(driveSim.getOdometry());
        SimulatedMecanumDrivetrainForFollower drivetrain = new SimulatedMecanumDrivetrainForFollower(Constants.driveConstants, driveSim);
        return new Follower(Constants.followerConstants, localizer, drivetrain, Constants.pathConstraints);
    }


    public static final double length = 144;
    public static final double width = 144;

}