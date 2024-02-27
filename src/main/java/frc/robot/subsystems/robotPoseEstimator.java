package frc.robot.subsystems;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;


public class robotPoseEstimator extends SubsystemBase
{
  private SwerveDrivePoseEstimator swervePose;
  private Swerve s_Swerve;

  public robotPoseEstimator(Swerve s_Swerve)
  {
    this.s_Swerve = s_Swerve;
    swervePose = new SwerveDrivePoseEstimator(frc.robot.Constants.Swerve.swerveKinematics, 
    s_Swerve.getYaw(), 
    s_Swerve.getModulePositions(), 
    new Pose2d());
  }

  @Override
  //update swerve pose estimator
  public void periodic()
  {
    //swervePose.update(s_Swerve.getYaw(), s_Swerve.getModulePositions()); //Commenting out to check for BUS Usage
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
