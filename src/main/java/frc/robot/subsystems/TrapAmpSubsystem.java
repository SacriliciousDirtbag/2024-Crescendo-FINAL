package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.eState;
import frc.robot.State.tState;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class TrapAmpSubsystem extends SubsystemBase {
    public CANSparkMax m_trapMotor;
    public CANSparkMax m_RightArmMotor;
    public CANSparkMax m_LeftArmMotor;
    public tState tState; //Spinner
    public eState eState; //Arm

    private double spinSpeed;
    private double spinCurrentLimit;

    private DutyCycleEncoder t_Encoder;
    private PIDController tPID;
    private ArmFeedforward tFeedforward;

    private double tPV; //curr position
    private double tSetPoint; //destination we want to go to

    //POSE PARAMETERS
    double toHome;
    double toTrap;
    double toAim; //Arbitrary value based on distance, shoots
    

    

    public TrapAmpSubsystem(){
        m_trapMotor = new CANSparkMax(frc.robot.Constants.AmpSystem.trapScorerID, MotorType.kBrushless);
        m_RightArmMotor = new CANSparkMax(frc.robot.Constants.AmpSystem.RightAimMotorID, MotorType.kBrushless);
        m_LeftArmMotor = new CANSparkMax(frc.robot.Constants.AmpSystem.LeftAimMotorID, MotorType.kBrushless);
        
        m_trapMotor.setIdleMode(IdleMode.kBrake);
        m_RightArmMotor.setIdleMode(IdleMode.kBrake);
        m_LeftArmMotor.setIdleMode(IdleMode.kBrake);

        tState = frc.robot.State.tState.STOP;


        
        t_Encoder = new DutyCycleEncoder(frc.robot.Constants.AmpSystem.ampEncoderID); //PWM Channel
        
        double ffP = 0;
        double ffI = 0;
        double ffD = 0;
        tPID = new PIDController(ffP, ffI, ffD);

        tFeedforward = new ArmFeedforward(0, 0.2, 0); //-0.15

        eState = frc.robot.State.eState.HOME;


        //ARM SETPOINTS
        toHome = 0; //TODO: calibrate Trap ARM Setpoints
        toTrap = 0; 
        toAim = 0; 


        //CANBUS USAGE
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_trapMotor, Usage.kVelocityOnly);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_RightArmMotor, Usage.kPositionOnly);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_LeftArmMotor, Usage.kPositionOnly);
    }

    private double tPos() {
        return t_Encoder.getAbsolutePosition() * 360;
    }



    @Override
    public void periodic(){
        
        m_trapMotor.set(spinSpeed);

        //ARM
        tPV = tPos();
        double tOutput = tPID.calculate(tPV, tSetPoint);
        m_RightArmMotor.set(tOutput);
        m_LeftArmMotor.set(tOutput);

        SmartDashboard.putNumber("Arm Encoder Rot:", tPos());

    }

    public double TPos(){
        return m_trapMotor.getEncoder().getPosition();
    }

    //Spinner
    public void goTState(tState state){
        if(state == frc.robot.State.tState.IN){
            spinSpeed = -0.75;
        }

        if(state == frc.robot.State.tState.OUT){
            spinSpeed = 0.75;
            
        }

        if(state == frc.robot.State.tState.STOP){
            spinSpeed = 0;

        }

    }

    //ARM SET SETPOINT
    public void setTSetPoint(double setpoint){
        tSetPoint = setpoint;
    }


    //Arm
    public void goEState(eState state){
        if(state == frc.robot.State.eState.HOME){
            tSetPoint = toHome;
            eState = frc.robot.State.eState.HOME;
            
        }

        if(state == frc.robot.State.eState.TRAP_POS){
            tSetPoint = toTrap;
            eState = frc.robot.State.eState.TRAP_POS;
            
        }

        if(state == frc.robot.State.eState.AIM_POS){
            tSetPoint = toAim;
            eState = frc.robot.State.eState.AIM_POS;

        }

    }

}
