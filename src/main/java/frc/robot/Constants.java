package frc.robot;

import java.util.Collections;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.PDPSim;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import frc.lib.util.COTSFalconSwerveConstants;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveSubsystem;

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
public final class Constants{
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double kSlipCurrentA = 50; // was 300

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double kSpeedAt12VoltsMps = 3;

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    private static final double kDriveGearRatio = 6.746031746031747;
    private static final double kSteerGearRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2;

    private static final boolean kSteerMotorReversed = true;
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final String kCANbusName = "1056_Canivore";
    private static final int kPigeonId = 13;


    static final double acceleration = 70; //was 100
    static final double strafeacceleration = 70;
    static final double turnacceleration = 70;
    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltage = 0.25;
    private static final double kDriveFrictionVoltage = 0.25;

    private static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withPigeon2Id(kPigeonId)
            .withCANbusName(kCANbusName);

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);


    // Front Left
    private static final int kFrontLeftDriveMotorId = 1;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 3;
    private static final double kFrontLeftEncoderOffset =  -0.195068359375;
    //-0.33642578125

    private static final double kFrontLeftXPosInches = 12;
    private static final double kFrontLeftYPosInches = 12;

    // Front Right
    private static final int kFrontRightDriveMotorId = 7;
    private static final int kFrontRightSteerMotorId = 8;
    private static final int kFrontRightEncoderId = 9;
    private static final double kFrontRightEncoderOffset = -0.3359375;


    private static final double kFrontRightXPosInches = 12;
    private static final double kFrontRightYPosInches = -12;

    // Back Left
    private static final int kBackLeftDriveMotorId = 4;
    private static final int kBackLeftSteerMotorId = 5;
    private static final int kBackLeftEncoderId = 6;
    private static final double kBackLeftEncoderOffset = -0.409423828125;

    private static final double kBackLeftXPosInches = -12;
    private static final double kBackLeftYPosInches = 12;

    // Back Right
    private static final int kBackRightDriveMotorId = 10;
    private static final int kBackRightSteerMotorId = 11;
    private static final int kBackRightEncoderId = 12;
    private static final double kBackRightEncoderOffset =  -0.088134765625;

    private static final double kBackRightXPosInches = -12;
    private static final double kBackRightYPosInches = -12;


    private static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
    private static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
    private static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);

    public static final SwerveSubsystem DriveTrain = new SwerveSubsystem(DrivetrainConstants, FrontLeft,
            FrontRight, BackLeft, BackRight);


    




    //22,23,24, 16, 17, 13 ON PWM

     //PWM 0 - Amp Spinner - 24
    //PWM 1 - Left Amp Arm - 16
    //PWM 2 - Right Amp Arm - 17
    //PWM 3 - Floor Intake - 13
    //PWM 4 - Right Feed Aim - 23
    //PWM 5 - Left Feed Aim - 22
    
    public static final class AutoConstants
    {
        public static final double driveKP = driveGains.kP;
        public static final double driveKI = driveGains.kI;
        public static final double driveKD = driveGains.kD;

        public static final double angleKP = steerGains.kP;
        public static final double angleKI = steerGains.kI;
        public static final double angleKD = steerGains.kD;

        public static final PIDController xDrivePID = new PIDController(driveKP, driveKI, driveKD);
        public static final PIDController yDrivePID = new PIDController(driveKP, driveKI, driveKD);
        public static final PIDController anglePID = new PIDController(angleKP, angleKI, angleKD);


    }

    public static final class IntakeSystem {
            
        public static final class IntakeWheel {
            public static final int frontMotorID = 3; //NeoPWM, 13
            public static final int wheelMotorID = 15; //Vortex, 15
            public static final int floorMotorID = 14; //Vortex, 14
        } 
    } 

    public static final class shooterAimingSystem {
        public static final int LeftAimID = 18; //Vortex
        public static final int RightAimID = 19; //Vortex
        

    }

    public static final class feederSubsystem{

        public static final int rightMotorID = 20; //Vortex
        public static final int leftMotorID = 21; //Vortex

        public static final int feederEncoderID = 1;  //DIO, 1
        

    }

    
    public static final class shooterSystem {
        public static final int LeftFlyWheelID = 5; //NeoPWM, CAN 22, PWM 5
        public static final int RightFlyWheelID = 4; //NeoPWM, CAN 23, PWM 4
    }
    

    public static final class AmpSystem {
        public static final int LeftAmpArmID = 16; //NeoPWM, 16, 1
        public static final int RightAmArmID = 17; //NeoPWM, 17, 2
        
        
        public static final int trapScorerID = 0; //NeoPWM, ID 24

        public static final int ampEncoderID = 0; //DIO, 0
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



//PID NOTES


/* 

{
  "drive": {

    "p": 1,
    "i": 0,
    "d": 0,
    "f": 0,
    "iz": 0
  },
  "angle": {
    "p": 4,
    "i": 1.25,
    "d": 0,
    "f": 0,
    "iz": 0
  }
}

*/ 
