package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.Command; //Class
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;


public class lbPlusB extends Command {
    public feederSubsystem s_feederSubsystem;
    public intakeSubsystem s_IntakeSubsystem;
    public TrapAmpSubsystem s_TrapAmpSubsystem;


    public lbPlusB(feederSubsystem feed, intakeSubsystem intake, TrapAmpSubsystem trap) {
        s_feederSubsystem = feed;
        s_IntakeSubsystem = intake;
        s_TrapAmpSubsystem = trap;
      
    }

     @Override
    public void initialize(){
        //"Left Bumper Input" (OLD | Feeds Through Index)
        // s_IntakeSubsystem.goIntakeWheelState(iState.OUT);
        // s_feederSubsystem.goIndexWheelState(sState.OUT, 0.8);

        //"Right Trigger Input"
        s_IntakeSubsystem.goIntakeWheelState(iState.IN);
        s_TrapAmpSubsystem.goTrapWheelState(tState.IN);

        //"B Input"
        //s_feederSubsystem.goAimWheelState(fState.IN);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
