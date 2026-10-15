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
public class PollenShooterSubsystem extends SubsystemBase {

    public static double pollenShooterKp = 0.01;
    public static double pollenShooterKi = 0;
    public static double pollenShooterKd = 0;
    public static boolean changePollenVelocityCoefficents = false;


    public static double holdPosition = 0;
    public static double shootPosition = .5;
    public double cpr;
    public MotorEx pollenShooterMotor;
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

    InterpLUT pollenSpeeds;

    private Servo pollenShootServo;

    public PollenShooterSubsystem(HardwareMap hardwareMap) {
        // specifying motor allows top rpm tp be read from motor
        if (!Configurables.doSimulation) {
            pollenShooterMotor = new MotorEx(hardwareMap, "pollen", Motor.GoBILDA.RPM_1150);

            maxMotorRPM = pollenShooterMotor.getMaxRPM();

            slf = new SlewRateLimiter(1000);
            maxShootRPM = maxMotorRPM * .9;
            minShootRPM = maxShootRPM / 2;
            cpr = pollenShooterMotor.getCPR();

            pollenShootServo = hardwareMap.get(Servo.class, "pollenShoot");
            pollenShootServo.setDirection(Servo.Direction.FORWARD);
            pollenShootServo.setPosition(holdPosition);
        } else {
            shooterMotorSim = new MotorSimulator(true, 1150);
            shooterMotorSim.setInverted(true);
            direction = shooterMotorSim.isInverted();
        }

        sff = new SimpleMotorFeedforward(.0, .9 / maxMotorRPM, 0);

        pidController = new PIDController(pollenShooterKp, pollenShooterKi, pollenShooterKd);

        //distance in inches, speeds in RPM
        pollenSpeeds = new InterpLUT() {{
            add(45.0, 3000.);
            add(40.0, 2800);
            add(35.0, 2500);
            add(30.0, 2000);
            add(25.0, 1800);
        }};
    }

    public void setVelocityCoefficients() {
        pidController.setPID(pollenShooterKp, pollenShooterKi, pollenShooterKd);
    }

    public void runPollenShooter(double pct) {
        if (!Configurables.doSimulation)
            pollenShooterMotor.set(pct);
        else shooterMotorSim.setPower(pct);
    }

    public Command jogPollenShooterCommand(double pct) {
        return Commands.runOnce(() -> runPollenShooter(pct), this);
    }

    public void stopPollenShooter() {
        if (!Configurables.doSimulation) {
            pollenShooterMotor.stopMotor();
            pollenShooterMotor.set(0);
        } else {
            shooterMotorSim.setPower(0);
        }
    }

    public Command stopPollenShooterCommand() {
        return Commands.runOnce(this::stopPollenShooter, this);
    }

    /**
     * Calculate the PIDF output from the controller and apply it to the motor
     * The F value is kf * target rpm
     * The controller setpoint is set by setTargetRPM()
     * The motor defaults to the raw power mode
     */
    public void runShooter2AtVelocity() {
        tst++;
        pidout = pidController.calculate(getMotorRPM());
        ff = sff.calculate(targetRPM);
        if (!Configurables.doSimulation)
            pollenShooterMotor.set(ff + pidout);
        else shooterMotorSim.setPower(ff + pidout);
    }

    public boolean pollenShooterAtSpeed() {
        if (!Configurables.doSimulation)
            return Math.abs(getTargetRPM() - getMotorRPM()) < getTargetRPM() / 10;
        else return true;
    }

    public double getVelocityError() {
        return pidController.getVelocityError();
    }

    public Command runShooter2AtVelocityCommand() {
        return Commands.run(this::runShooter2AtVelocity, this);
    }

    public double getMotorRPM() {
        if (!Configurables.doSimulation) {
            return pollenShooterMotor.getVelocity() * 60. / cpr;
        } else return shooterMotorSim.getVelocityRPM();
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

    public Command setTargetRPMCommand(double RPM) {
        return Commands.runOnce(() -> setTargetRPM(RPM));
    }

    public void changeTargetRPM(double val) {
        double tempRPM = getTargetRPM() + val;
        if (tempRPM > maxShootRPM) tempRPM = maxShootRPM;
        if (tempRPM < minShootRPM) tempRPM = minShootRPM;


        setTargetRPM(tempRPM);
    }

    public Command changeTargetRPMCommand(double val) {
        return Commands.runOnce(() -> changeTargetRPM(val));
    }

    public double getTargetSpeed(double inches) {
        return pollenSpeeds.get(inches);
    }

    public void positionPollenShooter(double position) {
        if (Configurables.doSimulation)
            pollenShootServo.setPosition(position);
    }

    public Command shootPollenCommand() {
        if (!Configurables.doSimulation)
            return Commands.runOnce(() -> positionPollenShooter(shootPosition));
        else return Commands.waitMillis(500);
    }

    public Command holdPollenCommand() {
        if (!Configurables.doSimulation)
            return Commands.runOnce(() -> positionPollenShooter(holdPosition));
        else return Commands.waitMillis(500);
    }
}

