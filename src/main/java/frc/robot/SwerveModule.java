package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.COTSFalconSwerveConstants.driveGearRatios;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    public double CANcoderInitTime = 0.0;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);
                

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID, "1056_Canivore");
        
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, "1056_Canivore");
        
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        resetToAbsolute();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, "1056_Canivore");
        
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        mDriveMotor.getConfigurator().setPosition(0.0);

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
        
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        anglePosition.Position = Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.setControl(anglePosition);
        
        //mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        lastAngle = angle;

        SmartDashboard.putNumber("Angle Position", anglePosition.Position);
        SmartDashboard.putNumber("Angle Degrees", lastAngle.getDegrees());
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getPosition().getValueAsDouble(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

      private void waitForCanCoder(){
        /*
         * Wait for up to 1000 ms for a good CANcoder signal.
         *
         * This prevents a race condition during program startup
         * where we try to synchronize the Falcon encoder to the
         * CANcoder before we have received any position signal
         * from the CANcoder.
         */
        for (int i = 0; i < 100; ++i) {
            angleEncoder.getAbsolutePosition();
            if (angleEncoder.getAbsolutePosition() != null) {
                break;
            }
            Timer.delay(0.010);            
            CANcoderInitTime += 10;
        }
    }

    public void resetToAbsolute(){ //TODO: check
        waitForCanCoder();
        double absolutePosition = Conversions.degreesToFalcon(getCANcoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        // mAngleMotor.setPosition(absolutePosition);
        setDesiredState(new SwerveModuleState(0,new Rotation2d(Units.degreesToRotations(absolutePosition))), false);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference), //TODO: NULL POINTER!!
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}