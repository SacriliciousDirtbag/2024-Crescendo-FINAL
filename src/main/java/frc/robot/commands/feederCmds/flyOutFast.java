package frc.robot.commands.feederCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class flyOutFast extends Command {
    public feederSubsystem s_feederSubsystem;

    public flyOutFast(feederSubsystem feed) {
        s_feederSubsystem = feed;
      
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goIndexWheelState(sState.OUT, 0.4); //TODO: Change fast index speed
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}