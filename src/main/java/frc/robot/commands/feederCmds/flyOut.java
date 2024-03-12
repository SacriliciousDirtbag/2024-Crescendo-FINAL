package frc.robot.commands.feederCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class flyOut extends Command {
    public feederSubsystem s_feederSubsystem;

    public flyOut(feederSubsystem feed) {
        s_feederSubsystem = feed;
      
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goIndexState(sState.OUT, 0.2);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}