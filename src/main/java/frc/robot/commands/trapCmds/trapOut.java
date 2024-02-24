package frc.robot.commands.trapCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.State.*;

public class trapOut extends Command {
    private TrapAmpSubsystem s_TrapAmpSubsystem;

    public trapOut(TrapAmpSubsystem trap) {
        this.s_TrapAmpSubsystem = trap;
      
    }

    @Override
    public void initialize() {
        s_TrapAmpSubsystem.goTState(tState.OUT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}