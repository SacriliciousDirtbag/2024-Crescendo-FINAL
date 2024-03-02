package frc.robot;

import java.util.Collections;
import java.util.List;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

//import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class AutonConstants
  {

    // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    // public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0;
    public static final double LEFT_Y_DEADBAND  = 0;
    public static final double RIGHT_X_DEADBAND = 0;
    public static final double TURN_CONSTANT    = 6;
  }    



    //22,23,24, 16, 17, 13 ON PWM

     //PWM 0 - Amp Spinner - 24
    //PWM 1 - Left Amp Arm - 16
    //PWM 2 - Right Amp Arm - 17
    //PWM 3 - Floor Intake - 13
    //PWM 4 - Right Feed Aim - 23
    //PWM 5 - Left Feed Aim - 22

    public static final class IntakeSystem {
            
        public static final class IntakeWheel {
            public static final int frontMotorID = 3; //NeoPWM, 13
            public static final int wheelMotorID = 14; //Vortex
            public static final int backMotorID = 15; //Vortex
        } 
    } 

    public static final class shooterAimingSystem {
        public static final int LeftAimID = 18; //Vortex
        public static final int RightAimID = 19; //Vortex
        

    }

    public static final class feederSubsystem{

        public static final int rightMotorID = 20; //Vortex
        public static final int leftMotorID = 21; //Vortex

        public static final int feederEncoderID = 0;  //DIO, 0
        

    }

    
    public static final class shooterSystem {
        public static final int LeftFlyWheelID = 5; //NeoPWM, 22
        public static final int RightFlyWheelID = 4; //NeoPWM, 23
    }
    

    public static final class AmpSystem {
        public static final int LeftAmpArmID = 1; //NeoPWM, 16
        public static final int RightAmArmID = 2; //NeoPWM, 17
        
        
        public static final int trapScorerID = 0; //NeoPWM, ID 24

        public static final int ampEncoderID = 1; //DIO, 
    }


    public static final class autoConfigs {
        //Adjustment
        public static final Translation2d[] backupLocations = {new Translation2d(-1,0.1)};
        public static final Pose2d backupEndLocation = new Pose2d(-2, 0.1, Rotation2d.fromDegrees(0)); //y was 0 

        //Route1 (Near 2nd Cube)
        public static final Translation2d[] route1Locations = {new Translation2d(-2,0)};
        //                                                                                          DO NOT CHANGE 
        public static final Pose2d route1EndLocation = new Pose2d(-2.3, 0.1, Rotation2d.fromDegrees(179.8)); //y was 180 //gyro offset 182

          //Pickup cube (move towards cube)
          public static final Translation2d[] pickupcubeLocations = {new Translation2d(-3.5,0.1)};
          public static final Pose2d pickupcubeLocation = new Pose2d(-2.5, 0.15, Rotation2d.fromDegrees(0));

        //Route2 (Scoring Hub)
        public static final Translation2d[] route2Locations = {new Translation2d(-4.8,0.1)};
        public static final Pose2d route2EndLocation = new Pose2d(0, 0.1, Rotation2d.fromDegrees(180));

        //Route3 (Balance)
        public static final Translation2d[] route3Locations = {new Translation2d(2.5, 0.15), new Translation2d(2.6, 0.1), new Translation2d(2.6, 1), new Translation2d(2.6, 2)};
        public static final Pose2d route3EndLocation = new Pose2d(0.3, 2, Rotation2d.fromDegrees(0));
    }

    public static final class cameraSettings
    {
        public static final double cameraHeight = Units.inchesToMeters(20.25);
        public static final double targetHeight1 = Units.inchesToMeters(43);
        public static final double cameraPitchRadians = Units.degreesToRadians(0);
    }


    public static final class PhotonConfig
    {
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d();
        public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
        public static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
            new Pose3d(), new Pose3d()));
    }
}

