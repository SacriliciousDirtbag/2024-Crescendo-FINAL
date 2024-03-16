package frc.robot.commands.trapCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class HOLDIT extends Command {
    private TrapAmpSubsystem s_TrapAmpSubsystem;
    private feederSubsystem s_feederSubsystem;

    public HOLDIT(TrapAmpSubsystem trap, feederSubsystem feed) {
        s_TrapAmpSubsystem = trap;
        s_feederSubsystem = feed;
      
    }

    @Override
    public void initialize() {
        s_TrapAmpSubsystem.goTrapArmState(eState.M_IDLE);
        s_feederSubsystem.goFeederArmState(aState.M_IDLE);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}