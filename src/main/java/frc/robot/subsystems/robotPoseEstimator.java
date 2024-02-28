package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;


public class robotPoseEstimator extends SubsystemBase
{
  private SwerveDrivePoseEstimator swervePose;
  private photonSubsystem s_PhotonCamera;
  private Swerve s_Swerve;

  public robotPoseEstimator(Swerve s_Swerve, photonSubsystem s_PhotonCamera)
  {
    this.s_Swerve = s_Swerve;
    this.s_PhotonCamera = s_PhotonCamera;
    swervePose = new SwerveDrivePoseEstimator(frc.robot.Constants.Swerve.swerveKinematics, 
    s_Swerve.getYaw(), 
    s_Swerve.getModulePositions(), 
    new Pose2d());
  }

  @Override
  //update swerve pose estimator
  public void periodic()
  {
    //swervePose.addVisionMeasurement(s_PhotonCamera.getVisionMeasurement().toPose2d(), s_PhotonCamera.getTimeStamp());
  }

  public void setCurrentPose(Pose2d newPose) {
    swervePose.resetPosition(
      s_Swerve.getYaw(),
      s_Swerve.getModulePositions(),
      newPose);
  }

   public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  public Pose2d getCurrentPose() {
    return swervePose.getEstimatedPosition();
  }

}
