package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.commands.AlignToPoseCommandSim;
import org.firstinspires.ftc.teamcode.simulator.commands.DriveSimCommand;
import org.firstinspires.ftc.teamcode.simulator.commands.DriveToPoseCommand;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.Arrays;

//@Autonomous(name = "Blank")
@TeleOp(name = "TestOpMode")

public class TestOpMode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;

    private Follower follower;

    MecanumDriveSubsystemSimulation driveSim;
    IntakeSubsystem intake;
    PollenShooterSubsystem pollenShooter;

    PathChain myPath;

    @Override
    public void initialize() {


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();




        driverGamepad = new GamepadEx(gamepad1);

        driveSim = new MecanumDriveSubsystemSimulation(this);
        telemetryM.addData("RC",driveSim.isRobotCentric());
        telemetryM.update(telemetry);
        follower = SimulatorConstants.createSimulatedFollower(driveSim);
        driveSim.setDefaultCommand(new DriveSimCommand(
                driveSim,
                () -> driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> driverGamepad.getRightX(), () -> true));

        intake = new IntakeSubsystem(this.hardwareMap);
        pollenShooter = new PollenShooterSubsystem(this.hardwareMap);

        myPath = buildBezierLine(new Pose(0, 0, 0), new Pose(10, 10, 0));


        GlobalData.setRedAlliance();

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new AlignToPoseCommandSim(driveSim, () -> driverGamepad.getLeftY(), () -> driverGamepad.getLeftX()));


        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        Commands.defer(() ->
                                new DriveToPoseCommand(driveSim, GlobalData.getActiveStartPose()), Arrays.asList()));


        // driverGamepad.getGamepadButton(GamepadKeys.Button.A)

//                //.whenPressed(shooter2.setMotorVelocityModeCommand())
//                .whenHeld(pollenShooter.jogPollenShooterCommand(0.5))
//                .whenReleased(pollenShooter.stopPollenShooterCommand());
//
//        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(pollenShooter.stopPollenShooterCommand());
//
//        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                    .whenHeld(intake.runIntakeCommand())
//                    .whenReleased(intake.stopIntakeCommand());
//
//        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(pollenShooter.runPollenShooterAtVelocityCommand());
//
//        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(pollenShooter.changeTargetRPMCommand(25));
//
//        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(pollenShooter.changeTargetRPMCommand(-25));


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();
//            if (PollenShooterSubsystem.changePollenVelocityCoefficents) {
//                pollenShooter.setVelocityCoefficients();
//                PollenShooterSubsystem.changePollenVelocityCoefficents = false;
//            }

//
//            telemetryM.addData("shooter2RPM", pollenShooter.getPollenShooterRPM());
//            telemetryM.addData("targetRPM", pollenShooter.getTargetRPM());
            telemetryM.addData("ROBCENTRIC", driveSim.isRobotCentric());
           telemetryM.update(telemetry);
        }
        reset();
    }

    PathChain buildBezierLine(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

}