package frc.robot.commands.intakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;

public class accumulateIn extends Command {
    private intakeSubsystem s_IntakeSubsystem;
    private feederSubsystem s_feederSubsystem;

    public accumulateIn(intakeSubsystem intake, feederSubsystem feeder) {
        s_IntakeSubsystem = intake;
        s_feederSubsystem = feeder;
      
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goFeederArmState(aState.HOME); //Arm Down
        s_IntakeSubsystem.goIntakeWheelState(iState.IN); //Feed In
        s_feederSubsystem.goIndexWheelState(sState.IN, 0.2); //Wheel In
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}