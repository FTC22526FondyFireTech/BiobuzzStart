package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.simulator.simulators.MotorSimulator;
import org.firstinspires.ftc.teamcode.utils.Configurables;

public class IntakeSubsystem extends SubsystemBase {

    public Motor intakeMotor;

    private MotorSimulator intakeMotorSim;

    private boolean direction;
    public static final double intakeSpeed = 0.77; // 1150 RPM * 0.77 (or / 77%) = 885.5
    protected double simDt = 0.01; // 10ms fixed physics step (100Hz)

    public IntakeSubsystem(HardwareMap hardwareMap) {
        if (!Configurables.doSimulation) {
            intakeMotor = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);
            intakeMotor.setInverted(true); // change to true if initially out-taking
            direction = intakeMotor.getInverted();
        } else {
            intakeMotorSim = new MotorSimulator(true,1150);
            intakeMotorSim.setInverted(true);
            direction = intakeMotorSim.isInverted();
        }


    }

    public void runIntake() {
        if (!Configurables.doSimulation)
            intakeMotor.set(intakeSpeed);
        else intakeMotorSim.setPower(intakeSpeed);
    }

    public void stopIntake() {
        if (!Configurables.doSimulation)
            intakeMotor.stopMotor();
        else intakeMotorSim.setPower(0);
    }

    public void invertIntake() {
        direction = !direction;
        intakeMotor.setInverted(direction);
    }

    public double getVelocityRPM() {
        if (!Configurables.doSimulation) {
            return intakeMotor.getCorrectedVelocity();
        } else {
            return intakeMotorSim.getVelocityRPM();
        }
    }

    public double getPower() {
        if (!Configurables.doSimulation) {
            return intakeMotor.getRawPower();
        } else {
            return intakeMotorSim.getPower();
        }
    }

    @Override
    public void periodic() {
        if (Configurables.doSimulation)
            intakeMotorSim.update(intakeMotorSim.getPower(), simDt);

    }

    public RunCommand runIntakeCommand() {
        return new RunCommand(
                this::runIntake, this);
    }

    public InstantCommand stopIntakeCommand() {
        return new InstantCommand(
                this::stopIntake, this);
    }

    public InstantCommand invertIntakeCommand() {
        return new InstantCommand(this::invertIntake);
    }

    public void showTelemetry(TelemetryManager telemetryM) {
        if (Configurables.showIntakeTelemetry) {
            telemetryM.addData("IntakePower", getPower());
            telemetryM.addData("IntakeRPM", getVelocityRPM());
        }
    }


}