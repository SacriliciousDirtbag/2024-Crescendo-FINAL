package frc.robot.commands.feederCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class feedStop extends Command {
    private feederSubsystem s_feederSubsystem;

    public feedStop(feederSubsystem feed) {
        s_feederSubsystem = feed;
      
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goFstate(fState.STOP);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}