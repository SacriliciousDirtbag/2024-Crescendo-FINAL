package frc.robot.commands.intakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class intakeIn extends Command {
    private intakeSubsystem s_IntakeSubsystem;
    private feederSubsystem s_feederSubsystem;

    public intakeIn(intakeSubsystem intake, feederSubsystem feeder) {
        s_IntakeSubsystem = intake;
        s_feederSubsystem = feeder;
      
    }

    @Override
    public void initialize() {
        s_IntakeSubsystem.goIstate(iState.IN);
        s_feederSubsystem.gosState(sState.IN);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}