package frc.robot.commands.feederCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.Constants.*;
import frc.robot.State.*;

public class setAimFar extends Command {
    public feederSubsystem s_feederSubsystem;

    public setAimFar(feederSubsystem feed) {
        s_feederSubsystem = feed;
      
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goAState(aState.AIM_FAR);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}