package frc.robot.commands.trapCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.State.*;

public class scoreAmp extends Command {
    private TrapAmpSubsystem s_TrapAmpSubsystem;

    public scoreAmp(TrapAmpSubsystem trap) {
        s_TrapAmpSubsystem = trap;
      
    }

    @Override
    public void initialize() {
        s_TrapAmpSubsystem.goTrapArmState(eState.AIM_POS);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}