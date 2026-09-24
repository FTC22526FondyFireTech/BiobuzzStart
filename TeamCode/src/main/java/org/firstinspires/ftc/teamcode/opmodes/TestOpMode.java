package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Shooter2Subsystem;
import org.firstinspires.ftc.teamcode.utils.Configurables;

//@Autonomous(name = "Blank")
@TeleOp(name = "TestOpMode")

public class TestOpMode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;
    IntakeSubsystem intake;
    Shooter2Subsystem shooter2;

    @Override
    public void initialize() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);

        driverGamepad = new GamepadEx(gamepad1);
        intake = new IntakeSubsystem(this.hardwareMap);
        shooter2 = new Shooter2Subsystem(this.hardwareMap);

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(shooter2.setMotorVelocityModeCommand())
                .whenPressed(shooter2.runShooter2Command(0.5));

        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(shooter2.stopShooter2Command());

        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(shooter2.setMotorJogModeCommand())
                .whenPressed(shooter2.jogShooter2Command(0.5))
                .whenReleased(shooter2.stopShooter2Command());

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                    .whenHeld(intake.runIntakeCommand())
                    .whenReleased(intake.stopIntakeCommand());


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            if(Configurables.changeVelocityCoefficents) {
                shooter2.setVelocityCoefficients();
                Configurables.changeVelocityCoefficents = false;
            }

            if(Configurables.changeFeedForwardCoefficents) {
                shooter2.setFeedForwardCoefficients();
                Configurables.changeFeedForwardCoefficents = false;
            }


            telemetryM.addData("shooter2RPM", shooter2.shooter2Motor.encoder.getCorrectedVelocity());
            telemetryM.update(telemetry);
        }
        reset();
    }

}