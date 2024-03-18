package frc.robot.commands.feederCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class flyStop extends Command {
    public feederSubsystem s_feederSubsystem;

    public flyStop(feederSubsystem feed) {
        s_feederSubsystem = feed;
      
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goIndexWheelState(sState.STOP, 0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}