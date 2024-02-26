package frc.robot.commands.intakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;

public class intakeIn extends Command {
    private intakeSubsystem s_IntakeSubsystem;

    public intakeIn(intakeSubsystem intake) {
        s_IntakeSubsystem = intake;
      
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