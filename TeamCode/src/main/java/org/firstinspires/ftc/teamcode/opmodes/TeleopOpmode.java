package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.simulator.SimulatorConstants;
import org.firstinspires.ftc.teamcode.simulator.commnands.DriveSimCommand;
import org.firstinspires.ftc.teamcode.simulator.drivetrains.MecanumDriveSubsystemSimulation;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Configurables;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

//@Autonomous(name = "Blank")
@TeleOp(name = "Teleop")
//@Disabled

public class TeleopOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;
    MecanumDriveSubsystem drive;
    private MecanumDriveSubsystemSimulation driveSim;
   // private IntakeSubsystem intake;
    private Follower follower;


    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);
        GlobalData.setBlueAlliance();
        driverGamepad = new GamepadEx(gamepad1);

        if (!Configurables.doSimulation) {
            drive = new MecanumDriveSubsystem(this.hardwareMap, new Pose());
            drive.setDefaultCommand(new DriveCommand(drive,
                    () -> driverGamepad.getLeftY(),
                    () -> -driverGamepad.getLeftX(),
                    () -> driverGamepad.getRightX()));
     //       intake = new IntakeSubsystem(this.hardwareMap);
        } else {
            driveSim = new MecanumDriveSubsystemSimulation(this);
            driveSim.setDefaultCommand(new DriveSimCommand(
                    driveSim,
                    () -> driverGamepad.getLeftY(),
                    () -> driverGamepad.getLeftX(),
                    () -> driverGamepad.getRightX(), () -> true));
        }


        if (!Configurables.doSimulation)
            follower = Constants.createFollower(this.hardwareMap);
        else
            follower = SimulatorConstants.createSimulatedFollower(driveSim);


        driverGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileActiveOnce(GlobalData.toggleAllianceCommand());

        if (!Configurables.doSimulation) {
            driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .whileActiveOnce(drive.resetPoseCommand());
        } else {
            driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                    .whileActiveOnce(driveSim.getOdometry().resetPoseCommand());
        }

        if (!Configurables.doSimulation) {
            driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whileActiveOnce(drive.setPoseCommand(new Pose(12, 12, Math.PI / 2)));
        } else {
            driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                    .whileActiveOnce(driveSim.getOdometry().setPoseCommand(new Pose(12, 12, Math.PI / 2)));

        }

//        if (!Configurables.doSimulation) {
//            driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                    .whenHeld(intake.runIntakeCommand())
//                    .whenReleased(intake.stopIntakeCommand());
//        }




    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            if (!Configurables.doSimulation) {
                drive.setRobotCentric(driverGamepad.getGamepadButton(
                        GamepadKeys.Button.RIGHT_BUMPER).get());
                drive.showTelemetry(telemetryM);
            } else {
                driveSim.setRobotCentric(driverGamepad.getGamepadButton(
                        GamepadKeys.Button.RIGHT_BUMPER).get());
                driveSim.showTelemetry(telemetryM);
            }

            if(Configurables.doSimulation)follower.update();

            //intake.showTelemetry(telemetryM);
            telemetryM.addData("IsBusy", follower.isBusy());
            //telemetryM.addData("RobotCentric", driveSim.isRobotCentric());

            telemetryM.update(telemetry);
        }
        reset();
    }

}