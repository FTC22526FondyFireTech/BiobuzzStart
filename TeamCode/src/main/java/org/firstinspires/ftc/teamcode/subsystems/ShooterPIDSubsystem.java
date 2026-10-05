package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.seattlesolvers.solverslib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.seattlesolvers.solverslib.gamepad.SlewRateLimiter;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.utils.Configurables;

public class ShooterPIDSubsystem extends SubsystemBase {

    public final double cpr;
    public MotorEx shooter2Motor;

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

    public ShooterPIDSubsystem(HardwareMap hardwareMap) {
        // specifying motor allows top rpm tp be read from motor
        shooter2Motor = new MotorEx(hardwareMap, "shooter2", Motor.GoBILDA.RPM_435);

        maxMotorRPM = shooter2Motor.getMaxRPM();

        slf = new SlewRateLimiter(1000);
        maxShootRPM = maxMotorRPM * .9;
        minShootRPM = maxShootRPM / 2;
        cpr = shooter2Motor.getCPR();

        sff = new SimpleMotorFeedforward(.0, .9 / maxMotorRPM, 0);

        pidController = new PIDController(Configurables.shooterKp, Configurables.shooterKi, Configurables.shooterKd);


    }

    public void setVelocityCoefficients() {
        pidController.setPID(Configurables.shooterKp, Configurables.shooterKi, Configurables.shooterKd);
    }

    public void runShooter2(double pct) {
        shooter2Motor.set(pct);
    }

    public Command jogShooter2Command(double pct) {
        return Commands.runOnce(() -> runShooter2(pct), this);
    }

    public void stopShooter2() {
        shooter2Motor.stopMotor();
        shooter2Motor.set(0);
    }

    public Command stopShooter2Command() {
        return Commands.runOnce(this::stopShooter2, this);
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
        shooter2Motor.set(ff + pidout);
    }

    public double getVelocityError() {
        return pidController.getVelocityError();
    }

    public Command runShooter2AtVelocityCommand() {
        return Commands.run(this::runShooter2AtVelocity, this);
    }

    public double getMotorRPM() {
        if (shooter2Motor != null)
            return shooter2Motor.getVelocity() * 60. / cpr;
        else return 0;
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

    public Command changeTargetRPMCommand(double val) {
        return Commands.runOnce(() -> changeTargetRPM(val));

    }
}
