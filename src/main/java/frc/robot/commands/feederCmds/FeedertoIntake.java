package frc.robot.commands.feederCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;

public class FeedertoIntake extends Command {
    private feederSubsystem s_feederSubsystem;
    private intakeSubsystem s_intakeSubsystem;

    public FeedertoIntake(feederSubsystem feed, intakeSubsystem intake) {
        s_feederSubsystem = feed;
        s_intakeSubsystem = intake;

    
    }

    @Override
    public void initialize() {
        s_feederSubsystem.goFeederArmState(aState.INTAKE_POS); //Move Arm to Intake
        s_intakeSubsystem.goIntakeWheelState(iState.IN); //Start moving intake
        s_feederSubsystem.goAimWheelState(fState.IN); //Start moving feeder wheel
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}