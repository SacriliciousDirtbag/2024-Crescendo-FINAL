package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.configs.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkFlex mAngleMotor;
    private CANSparkFlex mDriveMotor;
    
    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
   
    
    private final SparkPIDController driveController;
    private final SparkPIDController angleController;

    private CANcoderConfigurator angleConfigurator;
    private CANcoderConfiguration angleConfig;
     private CANcoder angleEncoder;

    private CANcoderConfigurator AbsoluteConfigurator;
    DeviceIdentifier DevIdentifier;

    public double CANcoderInitTime = 0.0;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */ 
        
        
        /* -- designating a canbus explicitly states where the motor controller is passing from, default seems to be roborio -- */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleConfigurator = angleEncoder.getConfigurator();
        configAngleEncoder();
        configAngleEncoder();
        configAngleEncoder();
        
        /* Angle Motor Config */ 
        mAngleMotor = new CANSparkFlex(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();
        configAngleMotor();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        driveController = mDriveMotor.getPIDController();
        configDriveMotor();
        configDriveMotor();
        configDriveMotor();

        lastAngle = getState().angle;

        // angleConfig.MagnetSensor.MagnetOffset = 0; //for each mod, reference to offset
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
          double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
          mDriveMotor.set(percentOutput);
        } else {
          driveController.setReference(CANcoderInitTime, CANSparkFlex.ControlType.kVelocity);
        }
      }

      private void setAngle(SwerveModuleState desiredState) {
        // Prevent rotating module if speed is less then 1%. Prevents jittering.
        Rotation2d angle =
            (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle;
    
        angleController.setReference(angle.getDegrees(), CANSparkFlex.ControlType.kPosition);
        lastAngle = angle;
      }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    // private void waitForCanCoder(){
    //     /*
    //      * Wait for up to 1000 ms for a good CANcoder signal.
    //      *
    //      * This prevents a race condition during program startup
    //      * where we try to synchronize the Falcon encoder to the
    //      * CANcoder before we have received any position signal
    //      * from the CANcoder.
    //      */
    //     for (int i = 0; i < 100; ++i) {
    //         angleEncoder.getAbsolutePosition();
    //         if (angleEncoder.getlast() == ErrorCode.OK) {
    //             break;
    //         }
    //         Timer.delay(0.010);            
    //         CANcoderInitTime += 10;
    //     }
    // }

    public void resetToAbsolute(){
        // waitForCanCoder();
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){       

        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());

        //CANSparkMaxUtil.setCANCoderBusUsage(angleEncoder, null);
        //angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){        
        mAngleMotor.restoreFactoryDefaults();
        //CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        mAngleMotor.setInverted(true);
        mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
        angleController.setP(Constants.Swerve.angleKP);
        angleController.setI(Constants.Swerve.angleKI);
        angleController.setD(Constants.Swerve.angleKD);
        angleController.setFF(Constants.Swerve.angleKF);
        mAngleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    
    }

    private void configDriveMotor(){        
        mDriveMotor.restoreFactoryDefaults();
        //CANSparkMaxUtil.setCANSparkMaxBusUsage(mDriveMotor, Usage.kAll);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
        driveController.setP(Constants.Swerve.angleKP);
        driveController.setI(Constants.Swerve.angleKI);
        driveController.setD(Constants.Swerve.angleKD);
        driveController.setFF(Constants.Swerve.angleKF);
        mDriveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
        mDriveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
        // return new SwerveModuleState(
        //     Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
        //     getAngle()
        // ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(7168, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }

}