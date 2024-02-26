package frc.robot.commands.intakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;

public class intakeIn extends Command {
    private intakeSubsystem s_IntakeSubsystem;

    public intakeIn(intakeSubsystem feed) {
        s_IntakeSubsystem = feed;
      
    }

    @Override
    public void initialize() {
        s_IntakeSubsystem.goIstate(iState.OUT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}