package org.firstinspires.ftc.teamcode.commands;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsysytem;
import org.firstinspires.ftc.teamcode.utils.GlobalData;

import java.util.List;

public class CheckForOdometryZoneTags extends CommandBase {
    private TelemetryManager telemetryM;
    private VisionSubsysytem vss;

    private int tagsZone;

    public CheckForOdometryZoneTags(VisionSubsysytem vss) {
        this.vss = vss;
    }

    @Override
    public void initialize() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }


    @Override
    public void execute() {
        List<LLResultTypes.FiducialResult> fr = vss.getFiducialResults();
        List<Integer> tagsSeen = vss.getTagsSeen(fr);
        tagsZone = vss.findZoneFromTags(tagsSeen);
        vss.setInOdometryZone(tagsZone == GlobalData.getCurrentOdometryZone());
    }


    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return vss.isInOdometryZone();
    }


}
