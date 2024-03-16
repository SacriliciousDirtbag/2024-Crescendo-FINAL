package frc.robot.commands.trapCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class hang extends Command {
    public feederSubsystem s_feederSubsystem;
    public TrapAmpSubsystem s_TrapAmpSubsystem;

    public hang(feederSubsystem feed, TrapAmpSubsystem trap) {
        s_feederSubsystem = feed;
        s_TrapAmpSubsystem = trap;
      
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goFeederArmState(aState.FLOAT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}