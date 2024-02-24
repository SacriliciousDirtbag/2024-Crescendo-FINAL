package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class Delay extends Command {
    public Timer clock;
    double start;
    double current;
    double seconds;

    public Delay (double seconds) {
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        start = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        current = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        if(current < start + seconds)
        {
            return false;
        } 
        return true;
    }

}
