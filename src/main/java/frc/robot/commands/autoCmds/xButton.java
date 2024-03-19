package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class xButton extends Command {
    public feederSubsystem s_feederSubsystem;

    public xButton(feederSubsystem feed) {
        s_feederSubsystem = feed;
      
    }

    @Override
    public void initialize() {
        //"X Input"
        s_feederSubsystem.goAimWheelState(fState.OUT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}