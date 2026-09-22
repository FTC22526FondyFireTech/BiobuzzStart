package org.firstinspires.ftc.teamcode.simulator.drivetrains;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;

/**
 * Drives a {@link MecanumDriveSubsystemSimulation} from a Pedro Pathing {@code Follower}.
 *
 * Pedro Pathing's real {@code com.pedropathing.ftc.drivetrains.Mecanum} can't be reused
 * directly here - its constructor requires a {@code HardwareMap} and real {@code DcMotorEx}
 * instances. {@link #calculateDrive} below is the same vector math as that class (pure math,
 * no hardware dependency), kept in lockstep so this produces identical wheel powers for the
 * same inputs. {@link #runDrive} writes those powers straight to the simulated motors, the
 * same way the real Mecanum writes straight to hardware - no additional forward/strafe/rotate
 * mixing happens here, since {@code calculateDrive} already produced final wheel powers.
 */
public class SimulatedMecanumDrivetrainForFollower extends Drivetrain {

    private final MecanumConstants constants;
    private final MecanumDriveSubsystemSimulation driveSim;

    public SimulatedMecanumDrivetrainForFollower(MecanumConstants constants, MecanumDriveSubsystemSimulation driveSim) {
        this.constants = constants;
        this.driveSim = driveSim;
        this.maxPowerScaling = constants.maxPower;
        this.nominalVoltage = constants.nominalVoltage;
        this.voltageCompensation = false; // no battery to simulate

        Vector copiedFrontLeftVector = constants.frontLeftVector.normalize();
        vectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2 * Math.PI - copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())};
    }

    @Override
    public void updateConstants() {
        maxPowerScaling = constants.maxPower;
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        if (correctivePower.getMagnitude() > maxPowerScaling) correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling) headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling) pathingPower.setMagnitude(maxPowerScaling);

        double[] wheelPowers = new double[4];
        Vector[] mecanumVectorsCopy = new Vector[4];
        Vector[] truePathingVectors = new Vector[2];

        if (correctivePower.getMagnitude() == maxPowerScaling) {
            truePathingVectors[0] = correctivePower.copy();
            truePathingVectors[1] = correctivePower.copy();
        } else {
            Vector leftSideVector = correctivePower.minus(headingPower);
            Vector rightSideVector = correctivePower.plus(headingPower);

            if (leftSideVector.getMagnitude() > maxPowerScaling || rightSideVector.getMagnitude() > maxPowerScaling) {
                double headingScalingFactor = Math.min(
                        findNormalizingScaling(correctivePower, headingPower, maxPowerScaling),
                        findNormalizingScaling(correctivePower, headingPower.times(-1), maxPowerScaling));
                truePathingVectors[0] = correctivePower.minus(headingPower.times(headingScalingFactor));
                truePathingVectors[1] = correctivePower.plus(headingPower.times(headingScalingFactor));
            } else {
                Vector leftSideVectorWithPathing = leftSideVector.plus(pathingPower);
                Vector rightSideVectorWithPathing = rightSideVector.plus(pathingPower);

                if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
                    double pathingScalingFactor = Math.min(
                            findNormalizingScaling(leftSideVector, pathingPower, maxPowerScaling),
                            findNormalizingScaling(rightSideVector, pathingPower, maxPowerScaling));
                    truePathingVectors[0] = leftSideVector.plus(pathingPower.times(pathingScalingFactor));
                    truePathingVectors[1] = rightSideVector.plus(pathingPower.times(pathingScalingFactor));
                } else {
                    truePathingVectors[0] = leftSideVectorWithPathing.copy();
                    truePathingVectors[1] = rightSideVectorWithPathing.copy();
                }
            }
        }

        truePathingVectors[0] = truePathingVectors[0].times(2.0);
        truePathingVectors[1] = truePathingVectors[1].times(2.0);

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            mecanumVectorsCopy[i] = vectors[i].copy();
            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[1].getYComponent()) / (mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent() - mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent() * truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent() * mecanumVectorsCopy[0].getYComponent()) / (mecanumVectorsCopy[0].getXComponent() * mecanumVectorsCopy[1].getYComponent() - mecanumVectorsCopy[1].getXComponent() * mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[3].getYComponent()) / (mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent() - mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent() * truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent() * mecanumVectorsCopy[2].getYComponent()) / (mecanumVectorsCopy[2].getXComponent() * mecanumVectorsCopy[3].getYComponent() - mecanumVectorsCopy[3].getXComponent() * mecanumVectorsCopy[2].getYComponent());

        double wheelPowerMax = Math.max(Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])), Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3])));
        if (wheelPowerMax > maxPowerScaling) {
            for (int i = 0; i < 4; i++) {
                wheelPowers[i] = (wheelPowers[i] / wheelPowerMax) * maxPowerScaling;
            }
        }

        return wheelPowers;
    }

    @Override
    public void breakFollowing() {
        driveSim.setDrivePowers(new double[]{0, 0, 0, 0});
    }

    @Override
    public void runDrive(double[] drivePowers) {
        // drivePowers order matches the real Mecanum's motor list: [leftFront, leftRear,
        // rightFront, rightRear]. MecanumDriveSubsystemSimulation.setDrivePowers() expects
        // [FL, FR, BL, BR], so reorder here.
        driveSim.setDrivePowers(new double[]{drivePowers[0], drivePowers[2], drivePowers[1], drivePowers[3]});
    }

    @Override
    public void startTeleopDrive() {
        // no brake/float behavior to simulate
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        // no brake/float behavior to simulate
    }

    @Override
    public double xVelocity() {
        return constants.xVelocity;
    }

    @Override
    public double yVelocity() {
        return constants.yVelocity;
    }

    @Override
    public void setXVelocity(double xMovement) {
        constants.setXVelocity(xMovement);
    }

    @Override
    public void setYVelocity(double yMovement) {
        constants.setYVelocity(yMovement);
    }

    @Override
    public double getVoltage() {
        return nominalVoltage; // no battery to simulate
    }

    @Override
    public String debugString() {
        return "SimulatedMecanumDrivetrain{maxPowerScaling=" + maxPowerScaling + "}";
    }
}
