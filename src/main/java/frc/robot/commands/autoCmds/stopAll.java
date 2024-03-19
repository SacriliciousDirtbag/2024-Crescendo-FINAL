package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.Command; //Class
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;


public class stopAll extends Command {
    public feederSubsystem s_feederSubsystem;
    public intakeSubsystem s_IntakeSubsystem;
    public TrapAmpSubsystem s_TrapAmpSubsystem;


    public stopAll(feederSubsystem feed, intakeSubsystem intake, TrapAmpSubsystem trap) {
        s_feederSubsystem = feed;
        s_IntakeSubsystem = intake;
        s_TrapAmpSubsystem = trap;
      
    }

     @Override
    public void initialize(){
        
        //Stop Everything EXCEPT for RAmp
        s_TrapAmpSubsystem.goTrapWheelState(tState.STOP);
        s_IntakeSubsystem.goIntakeWheelState(iState.STOP);
        s_feederSubsystem.goIndexWheelState(sState.STOP, 0);

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
