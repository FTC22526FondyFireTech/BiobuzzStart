package org.firstinspires.ftc.teamcode.simulator.simulators;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;

import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;

/**
 * Feeds a Pedro Pathing {@code Follower} from {@link OdometrySimulator} instead of real
 * hardware.
 *
 * OdometrySimulator's pose is kept current every physics tick by
 * {@link MecanumDriveSubsystemSimulation#periodic()},
 * independent of how often {@code Follower.update()} (and therefore this class's
 * {@link #update()}) gets called. So {@link #update()} only needs to derive a velocity
 * estimate from the pose delta since the last call - the same approach the real
 * {@code DriveEncoderLocalizer} uses (encoder delta over elapsed real time), just measured
 * in pose-space instead of ticks.
 */
public class SimulatedLocalizer implements Localizer {

    private final OdometrySimulator odometry;
    private final NanoTimer timer = new NanoTimer();

    private Pose startPose;
    private Pose lastPose;
    private Pose currentVelocity = new Pose();
    private double totalHeading = 0;

    public SimulatedLocalizer(OdometrySimulator odometry) {
        this.odometry = odometry;
        this.startPose = odometry.getRobotPose();
        this.lastPose = startPose;
    }

    @Override
    public Pose getPose() {
        return odometry.getRobotPose();
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getAsVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        // Per the Localizer contract this should only be called before any movement, so it's
        // equivalent to directly placing the simulated robot at the new pose.
        odometry.setPose(setStart.getX(), setStart.getY(), setStart.getHeading());
        startPose = setStart;
        lastPose = setStart;
    }

    @Override
    public void setPose(Pose setPose) {
        odometry.setPose(setPose.getX(), setPose.getY(), setPose.getHeading());
        lastPose = setPose;
    }

    @Override
    public void update() {
        double dt = timer.getElapsedTimeSeconds();
        timer.resetTimer();

        Pose pose = odometry.getRobotPose();
        if (dt > 0) {
            double dx = pose.getX() - lastPose.getX();
            double dy = pose.getY() - lastPose.getY();
            double dh = pose.getHeading() - lastPose.getHeading();
            // keep heading delta in [-pi, pi] so a wrap-around doesn't spike velocity
            while (dh > Math.PI) dh -= 2 * Math.PI;
            while (dh < -Math.PI) dh += 2 * Math.PI;

            currentVelocity = new Pose(dx / dt, dy / dt, dh / dt);
            totalHeading += dh;
        }
        lastPose = pose;
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    // These tick-to-unit multipliers are only meaningful for hardware localizers being
    // hand-tuned; the simulator's ticks are already converted internally by OdometrySimulator.
    @Override
    public double getForwardMultiplier() {
        return 1.0;
    }

    @Override
    public double getLateralMultiplier() {
        return 1.0;
    }

    @Override
    public double getTurningMultiplier() {
        return 1.0;
    }

    @Override
    public void resetIMU() {
        // no real IMU to reset
    }

    @Override
    public double getIMUHeading() {
        return odometry.getHeading();
    }

    @Override
    public boolean isNAN() {
        Pose pose = getPose();
        return Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading());
    }
}
