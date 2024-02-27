package frc.robot.commands.intakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;

public class intakeToAmp extends Command {
    private feederSubsystem s_feederSubsystem;
    private intakeSubsystem s_IntakeSubsystem;

    public intakeToAmp(feederSubsystem feed, intakeSubsystem intake) {
        s_feederSubsystem = feed;
        s_IntakeSubsystem = intake;
    
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