// package frc.robot.autos;

// import frc.robot.Constants;
// import frc.robot.subsystems.SwerveSubsystem;

// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


// public class middle extends SequentialCommandGroup {
    
//     public middle(SwerveSubsystem s_Swerve){
//         TrajectoryConfig config =
//             new TrajectoryConfig(
//                     3,
//                     1.5)
//                 .setKinematics(s_Swerve.getKinematics());

//         // An example trajectory to follow.  All units in meters.
//         Trajectory exampleTrajectory =

//             // Wrap around charging station //

//             //RIGHT SIDE POSITION
//             TrajectoryGenerator.generateTrajectory(
//                 // Start at the origin facing the +X direction
//                 new Pose2d(0, 0, new Rotation2d(0)),
//                 // Pass through these two interior waypoints, making a curved path
//                 List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
//                 // End 3 meters straight ahead of where we started, facing forward
//                 new Pose2d(2, -1, new Rotation2d(0)),
//                 config);

//             //MIDDLE POSITION
//             /*TrajectoryGenerator.generateTrajectory(
//                 // Start at the origin facing the +X direction
//                 new Pose2d(0, 0, new Rotation2d(0)),
//                 // Pass through these two interior waypoints, making a curved path
//                 List.of(new Translation2d(-1, 0), new Translation2d(-1.5, 0.1)), 
//                 // End 3 meters straight ahead of where we started, facing forward
//                 new Pose2d(-2.5, 0.1, new Rotation2d(0)),
//                 config);*/



//             //LEFT SIDE POSITION
//             /*TrajectoryGenerator.generateTrajectory(
//                 // Start at the origin facing the +X direction
//                 new Pose2d(0, 0, new Rotation2d(0)),
//                 // Pass through these two interior waypoints, making a curved path
//                 List.of(new Translation2d(-3.5, 0), new Translation2d(-2, 1.5), new Translation2d(-0.25, 1.5)), 
//                 // End 3 meters straight ahead of where we started, facing forward
//                     new Pose2d(-0.35, 1.5, new Rotation2d(0)),
//                     config);*/

            
//             // Drive on to charging station //
//             /*TrajectoryGenerator.generateTrajectory(
//                 // Start at the origin facing the +X direction
//                 new Pose2d(0, 0, new Rotation2d(0)),
//                 // Pass through these two interior waypoints, making a straight path
//                 List.of(new Translation2d(-2, 0), new Translation2d(-1.5, 0)),
//                 // End 3 meters straight ahead of where we started, facing forward
//                 new Pose2d(-1.25, 0, new Rotation2d(0)),
//                 config);*/
                
//         var thetaController =
//             new ProfiledPIDController(
//                 Constants.AutoConstants.anglePID, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         SwerveControllerCommand swerveControllerCommand =
//             new SwerveControllerCommand(
//                 exampleTrajectory,
//                 s_Swerve::getPose,
//                 s_Swerve.getKinematics(),
//                 new PIDController(Constants.AutoConstants.driveKD, 0, 0),
//                 new PIDController(Constants.AutoConstants.driveKD, 0, 0),
//                 thetaController,
//                 s_Swerve::setModuleStates,
//                 s_Swerve);


//         addCommands(
//             new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
//             swerveControllerCommand
//         );
//     }
// }