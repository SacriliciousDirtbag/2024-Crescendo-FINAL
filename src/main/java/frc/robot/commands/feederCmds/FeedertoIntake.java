package frc.robot.commands.feederCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;

public class FeedertoIntake extends Command {
    private feederSubsystem s_feederSubsystem;

    public FeedertoIntake(feederSubsystem feed, intakeSubsystem intake) {
        s_feederSubsystem = feed;
    
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goAState(aState.TRAP_POS); //Move Arm to Intake
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}