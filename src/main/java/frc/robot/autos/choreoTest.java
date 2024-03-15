// package frc.robot.autos;

// import frc.robot.Constants;
// import frc.robot.subsystems.SwerveSubsystem;


// import java.util.function.BooleanSupplier;

// import com.choreo.lib.Choreo;
// import com.choreo.lib.ChoreoControlFunction;
// import com.choreo.lib.ChoreoTrajectory;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.Subsystem;


// public class choreoTest extends SequentialCommandGroup {
//     private BooleanSupplier isBlueField;
    
//     public choreoTest(SwerveSubsystem s_Swerve){

//         // An example trajectory to follow.  All units in meters.
//         ChoreoTrajectory exampleTrajectory = Choreo.getTrajectory("Trajectory.1");
        
       
 
//         var thetaController = Constants.AutoConstants.anglePID;
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         ChoreoControlFunction choreoControl = Choreo.choreoSwerveController(
//             Constants.AutoConstants.xDrivePID, 
//             Constants.AutoConstants.yDrivePID, 
//             Constants.AutoConstants.anglePID);

//         Command choreoSwerveCommand = Choreo.choreoSwerveCommand(
//             exampleTrajectory, 
//             s_Swerve.getState().Pose,
//             choreoControl,
//             Constants.AutoConstants.xDrivePID,
//             Constants.AutoConstants.yDrivePID, 
//             thetaController, 
//             s_Swerve.getCurrentRobotChassisSpeeds(),
//             isBlueField, //Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
//             s_Swerve 


//             // (ChassisSpeeds speeds) -> {drivetrain.applyRequest(() -> drive.withVelocityX(MaxSpeed) // Drive forward with
//             //     // negative Y (forward)
//             //   .withVelocityY(MaxSpeed) // Drive left with negative X (left)
//             //   .withRotationalRate(MaxAngularRate) // Drive counterclockwise with negative X (left)
//             //   );},
//         );


//         addCommands(
//             new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
//             choreoSwerveCommand

//         );
//     }
// }