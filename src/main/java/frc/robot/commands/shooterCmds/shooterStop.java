package frc.robot.commands.shooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.State.*;

public class shooterStop extends Command {
    private shooterSubsystem s_shooterSubsystem;

    public shooterStop(shooterSubsystem s_shooterSubsystem) {
        this.s_shooterSubsystem = s_shooterSubsystem;
      
    }

    @Override
    public void initialize() {
        s_shooterSubsystem.stopWheels();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}