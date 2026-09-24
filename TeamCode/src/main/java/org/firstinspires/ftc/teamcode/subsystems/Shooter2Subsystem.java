package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.utils.Configurables;

public class Shooter2Subsystem extends SubsystemBase {

    public Motor shooter2Motor;

    private double P;
    private double I;
    private double D;

    public Shooter2Subsystem(HardwareMap hardwareMap) {
        shooter2Motor = new Motor(hardwareMap, "shooter2");
        shooter2Motor.setVeloCoefficients(
                Configurables.shooterKp,
                Configurables.shooterKi,
                Configurables.shooterKd);

        shooter2Motor.setFeedforwardCoefficients(
                Configurables.shooterKs,
                Configurables.shooterKv,
                Configurables.shooterKa);
    }

    public void setVelocityCoefficients() {
        shooter2Motor.setVeloCoefficients(
                Configurables.shooterKp,
                Configurables.shooterKi,
                Configurables.shooterKd);
    }

    public void setFeedForwardCoefficients() {
        shooter2Motor.setFeedforwardCoefficients(
                Configurables.shooterKs,
                Configurables.shooterKv,
                Configurables.shooterKa);
    }

    public void runShooter2(double pct) {
        shooter2Motor.set(pct);
    }

    public void stopShooter2() {
        shooter2Motor.stopMotor();
    }

    public void setMotorJogMode() {
        shooter2Motor.setRunMode(Motor.RunMode.RawPower);
    }
    public void setMotorVelocityMode() {
        shooter2Motor.setRunMode(Motor.RunMode.VelocityControl);
    }

    public Command runShooter2Command(double pct) {
        return new RunCommand(()->runShooter2(pct)).addRequirements(this);
    }

    public Command jogShooter2Command(double pct) {
        return new InstantCommand(()->runShooter2(pct));
    }

    public Command stopShooter2Command() {
        return new InstantCommand(this::stopShooter2).addRequirements(this);
    }

    public Command setMotorVelocityModeCommand() {
        return new InstantCommand(this::setMotorVelocityMode);
    }

    public Command setMotorJogModeCommand() {
        return new InstantCommand(this::setMotorJogMode);
    }
}
