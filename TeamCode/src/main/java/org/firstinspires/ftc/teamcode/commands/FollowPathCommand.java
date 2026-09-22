package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.paths.PathChain;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

/**
 * Autonomous path-following command. Hands a pre-built {@link PathChain} to the Follower
 * (via the drive subsystem) and finishes once the Follower reports it's no longer busy.
 * Compose these with SolversLib's SequentialCommandGroup / ParallelCommandGroup to build
 * an autonomous routine, e.g.:
 *
 * <pre>{@code
 * schedule(new SequentialCommandGroup(
 *     new FollowPathCommand(drive, scorePreload),
 *     new FollowPathCommand(drive, grabPickup1),
 *     new FollowPathCommand(drive, scorePickup1)
 * ));
 * }</pre>
 *
 * Note: if you add the optional {@code org.solverslib:pedroPathing} module, SolversLib ships
 * its own {@code com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand(follower, pathChain, holdEnd, maxPower)}
 * with a globalMaxPower decorator. That version takes the Follower directly and does not
 * declare a subsystem requirement, so nothing stops it from fighting a running DriveCommand
 * for the motors. This subsystem-scoped version trades that extra feature for correct
 * scheduler resource locking against {@link DriveCommand}; use whichever fits your OpMode.
 */
public class FollowPathCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final PathChain path;
    private final boolean holdEnd;

    public FollowPathCommand(MecanumDriveSubsystem drive, PathChain path) {
        this(drive, path, true);
    }

    public FollowPathCommand(MecanumDriveSubsystem drive, PathChain path, boolean holdEnd) {
        this.drive = drive;
        this.path = path;
        this.holdEnd = holdEnd;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
}
