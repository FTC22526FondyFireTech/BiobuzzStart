package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.simulator.simulators.MotorSimulator;
import org.firstinspires.ftc.teamcode.utils.Configurables;

@Configurable
public class NectarShooterSubsystem extends SubsystemBase {

    public static double nectarShooterKp = 0.01;
    public static double nectarShooterKi = 0;
    public static double nectarShooterKd = 0;
    public static boolean changeNectarVelocityCoefficents = false;


    public static double holdPosition = 0;
    public static double shootPosition = .5;

    public double cpr;
    public MotorEx nectarShooterMotor;
    private MotorSimulator shooterMotorSim;

    private boolean direction;
    public SimpleMotorFeedforward sff;

    public SlewRateLimiter slf;

    public PIDController pidController;

    private double targetRPM = 200;

    private double maxShootRPM;

    private double minShootRPM;

    private final double gearRatio = 13.7;
    public double pidout;

    public double ff;
    public int tst;

    public double maxMotorRPM;

    public InterpLUT nectarSpeeds;

    private Servo nectarShootServo;

    public boolean isRunNectarShooterAtVelocity() {
        return runNectarShooterAtVelocity;
    }

    public void setRunNectartShooterAtVelocity(boolean runNectarShooterAtVelocity) {
        this.runNectarShooterAtVelocity = runNectarShooterAtVelocity;
    }

    private boolean runNectarShooterAtVelocity;

    public NectarShooterSubsystem(HardwareMap hardwareMap) {
        // specifying motor allows top rpm tp be read from motor
        if (!Configurables.doSimulation) {
            nectarShooterMotor = new MotorEx(hardwareMap, "nectar", Motor.GoBILDA.RPM_1150);

            maxMotorRPM = nectarShooterMotor.getMaxRPM();

            slf = new SlewRateLimiter(1000);
            maxShootRPM = maxMotorRPM * .9;
            minShootRPM = maxShootRPM / 2;
            cpr = nectarShooterMotor.getCPR();
            nectarShootServo = hardwareMap.get(Servo.class, "nectarShoot");
            nectarShootServo.setDirection(Servo.Direction.FORWARD);
            nectarShootServo.setPosition(holdPosition);
        } else {
            shooterMotorSim = new MotorSimulator(true, 1150);
            shooterMotorSim.setInverted(true);
            direction = shooterMotorSim.isInverted();
        }

        sff = new SimpleMotorFeedforward(.0, .9 / maxMotorRPM, 0);

        pidController = new PIDController(nectarShooterKp, nectarShooterKi, nectarShooterKd);

//distance in inches, speeds in RPM
        nectarSpeeds = new InterpLUT() {{
            {
                {
                    add(45.0, 3000.);
                    add(40.0, 2800);
                    add(35.0, 2500);
                    add(30.0, 2000);
                    add(25.0, 1800);
                }
            }
            ;
        }};

    }

    public void setVelocityCoefficients() {
        pidController.setPID(nectarShooterKp, nectarShooterKi, nectarShooterKd);
    }

    public void runNectarShooter(double pct) {
        if (!Configurables.doSimulation)
            nectarShooterMotor.set(pct);
        else shooterMotorSim.setPower(pct);
    }

    public Command jogNectarShooterCommand(double pct) {
        return Commands.runOnce(() -> runNectarShooter(pct), this);
    }

    public void stopNectarShooter() {
        if (!Configurables.doSimulation) {
            nectarShooterMotor.stopMotor();
            nectarShooterMotor.set(0);
        } else {
            shooterMotorSim.setPower(0);
        }
    }

    public Command stopNectarShooterCommand() {
        return Commands.runOnce(this::stopNectarShooter, this);
    }

    /**
     * Calculate the PIDF output from the controller and apply it to the motor
     * The F value is kf * target rpm
     * The controller setpoint is set by setTargetRPM()
     * The motor defaults to the raw power mode
     */
    public void runNectarShooterAtVelocity() {
        tst++;
        pidout = pidController.calculate(getNectarShooterRPM());
        ff = sff.calculate(targetRPM);
        if (!Configurables.doSimulation)
            nectarShooterMotor.set(ff + pidout);
        else shooterMotorSim.setPower(ff + pidout);
    }

    public double getVelocityError() {
        return pidController.getVelocityError();
    }

    public Command runNectarShooterAtVelocityCommand() {
        return Commands.runOnce(() -> setRunNectartShooterAtVelocity(true));
    }

    public double getNectarShooterRPM() {
        if (!Configurables.doSimulation) {
            return nectarShooterMotor.getVelocity() * 60. / cpr;
        } else return shooterMotorSim.getVelocityRPM();
    }


    public boolean nectarShooterAtSpeed() {
        if (!Configurables.doSimulation)
            return Math.abs(getTargetRPM() - getNectarShooterRPM()) < getTargetRPM() / 10;
        else return true;
    }


    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Allows driver to adjust shoot speed
     * Needs 2 buttons +val increases speed -val decreases speed
     *
     * @param RPM
     */
    public void setTargetRPM(double RPM) {
        this.targetRPM = RPM;
        pidController.setSetPoint(targetRPM);
    }

    public void changeTargetRPM(double val) {
        double tempRPM = getTargetRPM() + val;
        if (tempRPM > maxShootRPM) tempRPM = maxShootRPM;
        if (tempRPM < minShootRPM) tempRPM = minShootRPM;


        setTargetRPM(tempRPM);
    }

    public Command changeNectarTargetRPMCommand(double val) {
        return Commands.runOnce(() -> changeTargetRPM(val));
    }

    public double getTargetRPMFromDistance(double inches) {
        return nectarSpeeds.get(inches);
    }
    public double getTargetSpeed(double inches) {
        return nectarSpeeds.get(inches);
    }

    public void positionNectarShooter(double position) {
        if (Configurables.doSimulation)
            nectarShootServo.setPosition(position);
    }

    public Command shootNectarCommand() {
        return Commands.runOnce(() -> positionNectarShooter(shootPosition));
    }

    public Command holdNectarCommand() {
        return Commands.runOnce(() -> positionNectarShooter(holdPosition));
    }

    @Override
    public void periodic() {
        if (runNectarShooterAtVelocity) {
            runNectarShooterAtVelocity();
        }
    }
}
