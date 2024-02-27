package frc.robot.commands.intakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;

public class intakeOut extends Command {
    private intakeSubsystem s_IntakeSubsystem;

    public intakeOut(intakeSubsystem intake) {
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