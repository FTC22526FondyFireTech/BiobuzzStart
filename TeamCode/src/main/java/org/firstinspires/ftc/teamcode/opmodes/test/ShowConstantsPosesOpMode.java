package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.utils.FieldConstants;

//@Autonomous(name = "Blank")
@TeleOp(name = "ShowPoses")
//@Disabled

public class ShowConstantsPosesOpMode extends CommandOpMode {

    TelemetryManager telemetryM;

    private int selectPoses;


    GamepadEx driverGamepad;
    public static Style robotLook = new Style("", "#3F51B5", 1.0);


    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);

        selectPoses = 0;

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(Commands.runOnce(this::incSelect));

        Drawing.init();

    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            Drawing.drawRobot(selectPose(selectPoses), robotLook);
            telemetryM.addData("SELPOS", selectPoses);
            telemetryM.update(telemetry);
        }
        reset();
    }

    private void incSelect() {
        selectPoses++;
        // if (selectPoses > 3) selectPoses = 0;
    }

    private Pose selectPose(int select) {

        switch (select) {

            case 0:
                return FieldConstants.redScoringStartPose;

            case 1:
                return FieldConstants.scoringWallFlowerApproachPose;

            case 2:
                return FieldConstants.scoringWallFlowerPickupPose;

            case 3:
                return FieldConstants.redScoringParkPose;

            case 4:
                return FieldConstants.blueAudienceStartPose;

            case 5:
                return FieldConstants.audienceWallFlowerApproachPose;

            case 6:
                return FieldConstants.blueallianceWallFlowerPickupPose;

            case 7:
                return FieldConstants.blueAudienceParkPose;

            default:
                selectPoses = 0;
                return new Pose();


        }

    }


}