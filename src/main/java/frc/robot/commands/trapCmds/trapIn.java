package frc.robot.commands.trapCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.State.*;

public class trapIn extends Command {
    private TrapAmpSubsystem s_TrapAmpSubsystem;

    public trapIn(TrapAmpSubsystem trap) {
        s_TrapAmpSubsystem = trap;
      
    }

    @Override
    public void initialize() {
        s_TrapAmpSubsystem.goTrapWheelState(tState.IN);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}