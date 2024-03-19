package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.Command; //Class
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;


public class leftBumper extends Command {
    public feederSubsystem s_feederSubsystem;
    public intakeSubsystem s_IntakeSubsystem;
    public TrapAmpSubsystem s_TrapAmpSubsystem;


    public leftBumper(feederSubsystem feed, intakeSubsystem intake, TrapAmpSubsystem trap) {
        s_feederSubsystem = feed;
        s_IntakeSubsystem = intake;
        s_TrapAmpSubsystem = trap;
      
    }

     @Override
    public void initialize(){
        //"Left Bumper Input"
        // s_IntakeSubsystem.goIntakeWheelState(iState.OUT);
        // s_feederSubsystem.goIndexWheelState(sState.OUT, 0.8);


        s_TrapAmpSubsystem.goTrapWheelState(tState.OUT);
        s_IntakeSubsystem.goIntakeWheelState(iState.AMP_IN);
        s_feederSubsystem.goIndexWheelState(sState.OUT, 0.8);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
