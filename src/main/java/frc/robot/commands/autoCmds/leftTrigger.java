package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.Command; //Class
import frc.robot.subsystems.feederSubsystem;
import frc.robot.State.*;


public class leftTrigger extends Command {
    public feederSubsystem s_feederSubsystem;

    public leftTrigger(feederSubsystem feed) {
        s_feederSubsystem = feed;
      
    }

     @Override
    public void initialize(){
        //"Left Trigger Input"
        s_feederSubsystem.goIndexWheelState(sState.IN, 0.2);
        s_feederSubsystem.goAimWheelState(fState.IN);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
