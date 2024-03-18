package frc.robot.commands.feederCmds;

import edu.wpi.first.wpilibj2.command.Command; //Class
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.State.*;


// Allows access to methods from Command inside intakeWithFeed
public class intakeWithFeed extends Command {
    public feederSubsystem s_feederSubsystem;
    public intakeSubsystem s_IntakeSubsystem;


    // Stores feed and intake subsystem into "feed" and "intake" variables
    public intakeWithFeed(feederSubsystem feed, intakeSubsystem intake) {
        s_feederSubsystem = feed;
        s_IntakeSubsystem = intake;
      
    }

    // changes speed of intake and feeder wheels
     @Override
    public void initialize(){
        s_IntakeSubsystem.goIntakeWheelState(iState.OUT);
        s_feederSubsystem.goIndexWheelState(sState.OUT, 0.2);

    }

    // removes command from command schedule
    @Override
    public boolean isFinished() {
        return true;
    }
}
