package frc.robot.autos;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.feederSubsystem;

import java.sql.Driver;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.feederCmds.*;
import frc.robot.commands.feederCmds.feedOut;
import frc.robot.Constants;
import frc.robot.State.fState;
import frc.robot.State.sState;

public class Blue1NoteAuto extends Command{
    Timer tim = new Timer();
    SwerveSubsystem s_Subsystem;
    feederSubsystem s_Feeder;
    SwerveRequest.FieldCentric test;
    ChassisSpeeds cSpeeds;
    


    public Blue1NoteAuto(SwerveSubsystem s_Swerve, feederSubsystem feeder)
    {
        s_Subsystem = s_Swerve;
        s_Feeder = feeder;
    }

    @Override
    public void initialize()
    {
        tim.reset();
        tim.start();
    }

    private double waitTime = 1.5;

    @Override
    public void execute(){
        double t= tim.get();

        s_Feeder.goAimWheelState(fState.OUT); //ramp wheels
        
        
        if(t > waitTime){
            s_Feeder.goIndexWheelState(sState.OUT, 0.4); //feed out
        }
    }

    @Override
    public void end(boolean isFinished)
    {
        s_Feeder.goIndexWheelState(sState.STOP, 0);
        s_Feeder.goAimWheelState(fState.STOP);
    }

    @Override
    public boolean isFinished(){
        if(tim.get() > 3.5)
        {
            return true;
        }
        return false;
    }
}
