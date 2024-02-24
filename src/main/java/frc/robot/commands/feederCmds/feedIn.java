package frc.robot.commands.feederCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class feedIn extends Command {
    private feederSubsystem s_feederSubsystem;

    public feedIn(feederSubsystem feed) {
        s_feederSubsystem = feed;
      
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goFstate(fState.OUT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}