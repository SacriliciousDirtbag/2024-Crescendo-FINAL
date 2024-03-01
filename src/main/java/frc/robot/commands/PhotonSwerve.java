package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.function.Supplier;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.photonSubsystem;


public class PhotonSwerve extends Command{
    photonSubsystem camera;
    Swerve s_Swerve;
    Supplier<Pose2d> supplier;
    public static final Transform3d TAG_TO_GOAL = new Transform3d(
        new Translation3d(1.5,0,0), new Rotation3d(0,0,Math.PI));
                                                                                                                                                                                                                                                                      
    TrapezoidProfile.Constraints theataTrap = new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    ProfiledPIDController theataPidController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, theataTrap);
    // PIDController xPidController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);

    TrapezoidProfile.Constraints xTrap = new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    TrapezoidProfile.Constraints yTrap = new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    ProfiledPIDController xPidController = new ProfiledPIDController(Constants.AutoConstants.kPXController,0,0,xTrap);
    ProfiledPIDController yPidController = new ProfiledPIDController(Constants.AutoConstants.kPYController,0,0,xTrap);
    //PIDController theataPidController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    // PIDController yPidController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
    Transform3d currentTransform3d;
    ChassisSpeeds chassisSpeeds;
    SwerveModuleState[] swerveModuleStates;

    double xCalculation;
    double yCalculation;
    double theataCaluclation;
    double angle;
    Pose3d targetPose;


    public PhotonSwerve(photonSubsystem camera, Swerve s_Swerve, Supplier<Pose2d> poseProvider)
    {
        this.camera = camera;
        this.s_Swerve = s_Swerve;
        this.supplier = poseProvider;
        xPidController.setTolerance(0.1);
        yPidController.setTolerance(0.1);
        theataPidController.enableContinuousInput(-Math.PI, Math.PI);
        theataPidController.setTolerance(Units.degreesToRadians(1));
    }

    @Override
    public void initialize() 
    {
        theataPidController.reset(s_Swerve.getGyroYaw().getRadians());
        var robotPose = supplier.get();
        // xPidController.setSetpoint(1);
        // yPidController.setSetpoint(0);
        xPidController.reset(robotPose.getX());
        yPidController.reset(robotPose.getY());
        //s_Swerve.zeroGyro(); Might be initialized before s_swerve? + Swerve Subsystem resets gyro on startup
        theataPidController.reset(robotPose.getRotation().getRadians());
    }

    @Override 
    public void execute()
    {
        // currentTransform3d = camera.getTransform3d();
        // angle = camera.getYaw(); 
        // theataCaluclation = -theataPidController.calculate(s_Swerve.gyro.getYaw().getValue()  % 360- angle); //Units.degreesToRadians(angle - s_Swerve.getYaw().getDegrees())
        // //if camera > gyro
        // if((Math.abs(s_Swerve.gyro.getYaw().getValue()) - Math.abs(angle)) < 0){ //Added Math.abs to both so neither are impacted by negatives
        //     theataCaluclation = -theataPidController.calculate(-(s_Swerve.gyro.getYaw().getValue() % 360 - angle));
        // }
        var robotPose2d = supplier.get();
        var robotPose = new Pose3d(robotPose2d.getX(), 
        robotPose2d.getY(), 0.0, new Rotation3d(0,0, robotPose2d.getRotation().getRadians()));
        
        var cameraPose = robotPose.transformBy(Constants.PhotonConfig.ROBOT_TO_CAMERA);
        var camToTarget = camera.getTransform3d();
        var targetPose = cameraPose.transformBy(camToTarget);
        var goal = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        // xCalculation = -xPidController.calculate(currentTransform3d.getX());
        // yCalculation = -yPidController.calculate(currentTransform3d.getY()); 
        xPidController.setGoal(goal.getX());
        yPidController.setGoal(goal.getY());
        theataPidController.setGoal(goal.getRotation().getRadians());


        xCalculation = xPidController.calculate(goal.getX());
        yCalculation = yPidController.calculate(goal.getY());
        theataCaluclation = theataPidController.calculate(goal.getRotation().getRadians());
        if (xPidController.atGoal()) {
            xCalculation = 0;
        }
        if (yPidController.atGoal()) {
            xCalculation = 0;
        }

        if (theataPidController.atGoal()) {
            theataCaluclation = 0;
        }
        xCalculation = xPidController.calculate(goal.getX());
        yCalculation = yPidController.calculate(goal.getY());
        theataCaluclation = theataPidController.calculate(goal.getRotation().getRadians());


        chassisSpeeds = new ChassisSpeeds(xCalculation,yCalculation,theataCaluclation);
        swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.AutoConstants.kMaxSpeedMetersPerSecond);
        s_Swerve.setModuleStates(swerveModuleStates);
    }

    @Override
    public void end(boolean isTrue)
    {
        SwerveModuleState[] stop = new SwerveModuleState[4];
        for(int i = 0; i < stop.length; i++)
        {
            stop[i] = new SwerveModuleState();
        }
        s_Swerve.setModuleStates(stop);
    }

    @Override
    public boolean isFinished()
    {
        // if(yPidController.atSetpoint())
        // {
        //     yPidController.reset();
        // }
        // if(xPidController.atSetpoint())
        // {
        //     xPidController.reset();
        // }
        // // if(theataPidController.atSetpoint())
        // // {
        // //     theataPidController.reset();
        // // }
        return false;
    }
}
