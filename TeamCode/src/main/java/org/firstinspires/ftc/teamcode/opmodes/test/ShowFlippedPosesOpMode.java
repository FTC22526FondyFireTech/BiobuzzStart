package org.firstinspires.ftc.teamcode.opmodes.test;

import com.bylazar.configurables.annotations.Configurable;
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
@TeleOp(name = "ShowFlippedPoses")
//@Disabled
@Configurable
public class ShowFlippedPosesOpMode extends CommandOpMode {

    TelemetryManager telemetryM;

    public static int selectPoses;


    GamepadEx driverGamepad;
    public static Style robotLook = new Style("", "#3F51B5", 1.0);
    private static final Style redLook = new Style("", "#FF0000", 1.0);
    private static final Style blueLook = new Style("", "#0000FF", 1.0);


    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);

        selectPoses = 0;

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(Commands.runOnce(this::incSelectMinus));

        driverGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(Commands.runOnce(this::incSelectPlus));

        Drawing.init();

    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();
            //  if (selectPoses <= 4 || selectPoses >= 10 && selectPoses <= 13 || selectPoses == 18)
            Drawing.drawRobot(selectPose(selectPoses), redLook);
            Drawing.sendPacket();
//            else
//                Drawing.drawRobot(selectPose(selectPoses), blueLook);

            telemetryM.addData("SELPOS", selectPoses);
            telemetryM.update(telemetry);
        }
        reset();
    }

    private void incSelectPlus() {
        selectPoses++;
        // if (selectPoses > 3) selectPoses = 0;
    }

    private void incSelectMinus() {
        selectPoses--;
        // if (selectPoses > 3) selectPoses = 0;
    }

    private Pose selectPose(int select) {

        switch (select) {

            case 0:
                return FieldConstants.redScoringLoHiveStartPose;

            case 1:
                return FieldConstants.toBlue(FieldConstants.redScoringLoHiveStartPose);

            case 2:
                return FieldConstants.scoringWallFlowerApproachPose;

            case 3:
                return FieldConstants.toBlue(FieldConstants.scoringWallFlowerApproachPose);
            case 4:
                return FieldConstants.scoringWallFlowerPickupPose;

            case 5:
                return FieldConstants.toBlue(FieldConstants.scoringWallFlowerPickupPose);

            case 6:
                return FieldConstants.redScoringSecondShootPose;
            case 7:
                return FieldConstants.toBlue(FieldConstants.redScoringSecondShootPose);

            case 8:
                return FieldConstants.redScoringParkPose;
            case 9:
                return FieldConstants.toBlue(FieldConstants.redScoringParkPose);

            case 10:
                return FieldConstants.blueAudienceLoHiveStartPose;

            case 11:
                return FieldConstants.toBlue(FieldConstants.blueAudienceLoHiveStartPose);

            case 12:
                return FieldConstants.audienceWallFlowerApproachPose;

            case 13:
                return FieldConstants.toBlue(FieldConstants.audienceWallFlowerApproachPose);

            case 14:
                return FieldConstants.audienceWallFlowerPickupPose;
            case 15:
                return FieldConstants.toBlue(FieldConstants.audienceWallFlowerPickupPose);

            case 16:
                return FieldConstants.blueAudienceSecondShootPose;

            case 17:
                return FieldConstants.toBlue(FieldConstants.blueAudienceSecondShootPose);


            case 18:
                return FieldConstants.blueAudienceParkPose;

            case 19:
                return FieldConstants.toBlue(FieldConstants.blueAudienceParkPose);


//            case 10:
//                return FieldConstants.redAudienceHiHiveStartPose;
//            case 11:
//                return FieldConstants.redAllianceWallFlowerApproachPose;
//            case 12:
//                return FieldConstants.redAllianceWallFlowerPickupPose;
//            case 13:
//                return FieldConstants.redAudienceParkPose;
//            case 14:
//                return FieldConstants.blueScoringHiHiveStartPose;
//            case 15:
//                return FieldConstants.blueAllianceWallFlowerApproachPose;
//            case 16:
//                return FieldConstants.blueAllianceWallFlowerPickupPose;
//            case 17:
//                return FieldConstants.blueAudienceParkPose;
//            case 18:
//                return FieldConstants.scoringWallFlowerApproachControlPointPose;
//            case 19:
//                return FieldConstants.auidenceWallFlowerApproachControlPointPose;
//

            case 100:
                return FieldConstants.tag30;

            default:
                selectPoses = 0;
                return new Pose();


        }

    }


}