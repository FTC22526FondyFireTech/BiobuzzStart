package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;

//@Autonomous(name = "Blank")
@TeleOp(name = "TestOpMode")

public class TestOpMode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;
    IntakeSubsystem intake;
    PollenShooterSubsystem pollenShooter;

    @Override
    public void initialize() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);

        driverGamepad = new GamepadEx(gamepad1);
        intake = new IntakeSubsystem(this.hardwareMap);
        pollenShooter = new PollenShooterSubsystem(this.hardwareMap);

        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                //.whenPressed(shooter2.setMotorVelocityModeCommand())
                .whenHeld(pollenShooter.jogPollenShooterCommand(0.5))
                .whenReleased(pollenShooter.stopShooter2Command());

        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(pollenShooter.stopShooter2Command());

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                    .whenHeld(intake.runIntakeCommand())
                    .whenReleased(intake.stopIntakeCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(pollenShooter.runShooter2AtVelocityCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(pollenShooter.changeTargetRPMCommand(25));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(pollenShooter.changeTargetRPMCommand(-25));


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            if(PollenShooterSubsystem.changePollenVelocityCoefficents) {
                pollenShooter.setVelocityCoefficients();
                PollenShooterSubsystem.changePollenVelocityCoefficents = false;
            }


            telemetryM.addData("shooter2RPM", pollenShooter.getMotorRPM());
            telemetryM.addData("targetRPM", pollenShooter.getTargetRPM());
            telemetryM.update(telemetry);
        }
        reset();
    }

}