package org.firstinspires.ftc.teamcode.utils;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.PollenShooterSubsystem;

public class AutoCommands {



    private final PollenShooterSubsystem pss;

    private double startShootRPM = 1500;





    public AutoCommands(PollenShooterSubsystem pss){
        this.pss = pss;

    }


    public Command autoShootCommand() {
        return Commands.sequence(
                pss.setTargetRPMCommand(startShootRPM),
                pss.runPollenShooterAtVelocityCommand(),
                new WaitUntilCommand(pss::pollenShooterAtSpeed),
                pss.shootPollenCommand(),
                new WaitCommand(2000),
                pss.holdPollenCommand(),
                pss.stopPollenShooterCommand());

    }
}
