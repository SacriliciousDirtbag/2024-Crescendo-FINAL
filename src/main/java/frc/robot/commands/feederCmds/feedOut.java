package frc.robot.commands.feederCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class feedOut extends Command {
    public feederSubsystem s_feederSubsystem;

    public feedOut(feederSubsystem feed) {
        s_feederSubsystem = feed;
      
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goAimWheelState(fState.OUT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}