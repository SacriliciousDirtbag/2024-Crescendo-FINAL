package frc.robot.commands.intakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;

public class intakeStop extends Command {
    private intakeSubsystem s_IntakeSubsystem;

    public intakeStop(intakeSubsystem intake) {
        s_IntakeSubsystem = intake;
      
    }

    @Override
    public void initialize() {
        s_IntakeSubsystem.goIntakeWheelState(iState.STOP);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}