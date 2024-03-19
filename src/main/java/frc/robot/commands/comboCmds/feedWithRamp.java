package frc.robot.commands.comboCmds;

import edu.wpi.first.wpilibj2.command.Command; //Class
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.State.*;


// Allows access to methods from Command inside intakeWithFeed
public class feedWithRamp extends Command {
    public feederSubsystem s_feederSubsystem;
    public TrapAmpSubsystem s_TrapAmpSubsystem;



    // Stores feed and intake subsystem into "feed" and "intake" variables
    public feedWithRamp(feederSubsystem feed, TrapAmpSubsystem trap) {
        s_feederSubsystem = feed;
        s_TrapAmpSubsystem = trap;
      
    }

    // changes speed of intake and feeder wheels
     @Override
    public void initialize(){
        s_feederSubsystem.goIndexWheelState(sState.IN, 0.2);
        s_feederSubsystem.goAimWheelState(fState.IN);
    }

    // removes command from command schedule
    @Override
    public boolean isFinished() {
        return true;
    }
}
