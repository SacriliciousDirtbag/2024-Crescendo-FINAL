package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class ChoreoTest2 extends SequentialCommandGroup {
    private PIDController xPidController = Constants.AutoConstants.xDrivePID;
    private PIDController yPidController = Constants.AutoConstants.yDrivePID;
    private PIDController theataPidController = Constants.AutoConstants.anglePID;
    
    private ChoreoTrajectory test = Choreo.getTrajectory("Trajectory.1");
    private ChoreoControlFunction choreoControl = Choreo.choreoSwerveController(xPidController,yPidController,theataPidController);
    private Command choreoSwerveCommand;
    private SwerveSubsystem s_Swerve;
    private Supplier<Pose2d> robotPoseSupplier;
    private BooleanSupplier trajectoryMirroredSupplier;
    private Consumer<ChassisSpeeds> chassisSupplier;


    public ChoreoTest2(SwerveSubsystem s_Swerve){
        this.s_Swerve = s_Swerve;
        choreoSwerveCommand = Choreo.choreoSwerveCommand(
            test, 
            robotPoseSupplier, 
            choreoControl, 
            chassisSupplier, 
            trajectoryMirroredSupplier,
            s_Swerve);
    }



}