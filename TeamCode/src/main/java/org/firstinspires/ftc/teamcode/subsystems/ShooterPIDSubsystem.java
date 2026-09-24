package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.utils.Configurables;

public class ShooterPIDSubsystem extends SubsystemBase {

    public Motor shooter2Motor;
    PIDFController shooterController;
    private double targetRPM = 500;
    private final double maxShootRPM = getMotorRPM() * .8;

    public ShooterPIDSubsystem(HardwareMap hardwareMap) {
        //specifying motor allows top rpm tp be read from motor
        shooter2Motor = new Motor(hardwareMap, "shooter2", Motor.GoBILDA.RPM_1150);
        shooterController = new PIDFController(Configurables.shooterKp, Configurables.shooterKi, Configurables.shooterKd, Configurables.shooterKf);
        shooterController.setSetPoint(targetRPM);
    }

    public void setVelocityCoefficients() {
        shooterController.setPIDF(
                Configurables.shooterKp,
                Configurables.shooterKi,
                Configurables.shooterKd,
                Configurables.shooterKf);
    }

    public void runShooter2(double pct) {
        shooter2Motor.set(pct);
    }

    public Command jogShooter2Command(double pct) {
        return Commands.runOnce(() -> runShooter2(pct), this);
    }

    public void stopShooter2() {
        shooter2Motor.stopMotor();
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
        double pidout = shooterController.calculate(getMotorRPM());
        shooter2Motor.set(pidout);
    }

    public double getVelocityError() {
        return shooterController.getVelocityError();
    }

    public Command runShooter2AtVelocityCommand() {
        return Commands.run(this::runShooter2AtVelocity, this);
    }

    public double getMotorRPM() {
        return shooter2Motor.getRate();
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Allows driver to adjust shoot speed
     * Needs 2 buttons +val increases speed -val decreases speed
     * @param RPM
     */
    public void setTargetRPM(double RPM) {
        double temp = Math.signum(RPM);
        if (RPM > maxShootRPM)
            RPM = maxShootRPM;
        this.targetRPM = RPM;
        shooterController.setSetPoint(targetRPM);
    }

    public void changeTargetRPM(double val) {
        double tempRPM = getTargetRPM() + val;
        setTargetRPM(tempRPM);
    }
}
