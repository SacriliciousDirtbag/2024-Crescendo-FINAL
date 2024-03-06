package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.eState;
import frc.robot.State.tState;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

public class TrapAmpSubsystem extends SubsystemBase {
    public PWMSparkMax m_trapMotor;
    public PWMSparkMax m_RightArmMotor;
    public PWMSparkMax m_LeftArmMotor;
    
    public tState tState; //Spinner
    public eState eState; //Arm

    private double spinSpeed = 0;
    private double spinCurrentLimit;

    private DutyCycleEncoder t_Encoder;
    private PIDController tPID;
    private ArmFeedforward tFeedforward;

    private double tPV; //curr position
    private double tSetPoint; //destination we want to go to

    //POSE PARAMETERS
    double MIN;
    double toHome;
    double toTrap;
    double toAim; //Arbitrary value based on distance, shoots
    double MAX;
    
    

    public TrapAmpSubsystem(){
        m_trapMotor = new PWMSparkMax(frc.robot.Constants.AmpSystem.trapScorerID);
        m_RightArmMotor = new PWMSparkMax(frc.robot.Constants.AmpSystem.RightAmArmID);
        m_LeftArmMotor = new PWMSparkMax(frc.robot.Constants.AmpSystem.LeftAmpArmID);
        
        //m_trapMotor.setIdleMode(IdleMode.kBrake);
       // m_RightArmMotor.setIdleMode(IdleMode.kBrake);
        //m_LeftArmMotor.setIdleMode(IdleMode.kBrake);

        tState = frc.robot.State.tState.STOP;


        
        t_Encoder = new DutyCycleEncoder(frc.robot.Constants.AmpSystem.ampEncoderID); //PWM Channel
        
        double ffP = 0.025; //TODO: Tune PID
        double ffI = 0;
        double ffD = 0;
        tPID = new PIDController(ffP, ffI, ffD);

        tFeedforward = new ArmFeedforward(0, 0, 0); //-0.15

        //eState = frc.robot.State.eState.HOME;


        //ARM SETPOINTS
        MIN = 8.42;
        toHome = 0; //TODO: calibrate Trap ARM Setpoints
        toTrap = 0; 
        toAim = 0; 
        MAX = 161.54;


        m_RightArmMotor.setInverted(true);

        //FAILSAFE
        m_LeftArmMotor.disable();
        m_RightArmMotor.disable();
    }

    private double tPos() {
        return t_Encoder.getAbsolutePosition() * 360;
    }

     @Override
    public void periodic(){

        m_trapMotor.set(spinSpeed);

        //ARM
        tPV = tPos();
        double tOutput = tPID.calculate(tPV, 90);
        m_RightArmMotor.set(0.1);
        m_LeftArmMotor.set(0.1);

        SmartDashboard.putNumber("Trap Arm Encoder Rot:",tPV); //Measured in Degrees
        SmartDashboard.putNumber("Trap Encoder DIO#", t_Encoder.getSourceChannel());
        SmartDashboard.putNumber("T Setpoint", 90);
        SmartDashboard.putNumber("T Output", tOutput);

    }


    //TRAP AMP Spinner
    public void goTState(tState state){
        if(state == frc.robot.State.tState.IN){
            spinSpeed = 1;
            tState = frc.robot.State.tState.IN;
        }

        if(state == frc.robot.State.tState.OUT){
            spinSpeed = -1;
             tState = frc.robot.State.tState.OUT;
            
        }

        if(state == frc.robot.State.tState.STOP){
            spinSpeed = 0;
             tState = frc.robot.State.tState.STOP;

        }

    }

    //ARM SET SETPOINT
    public void setTSetPoint(double setpoint){
        tSetPoint = setpoint;
    }

    //ARM GET SETPOINT
    public double getTSetPoint(){
        return tSetPoint;
    }


    //Arm
    public void goEState(eState state){
        if(state == frc.robot.State.eState.HOME){
            setTSetPoint(toHome);
            eState = frc.robot.State.eState.HOME;
            
        }

        if(state == frc.robot.State.eState.TRAP_POS){
            setTSetPoint(toTrap);
            eState = frc.robot.State.eState.TRAP_POS;
            
        }

        if(state == frc.robot.State.eState.AIM_POS){
            setTSetPoint(toAim);
            eState = frc.robot.State.eState.AIM_POS;

        }

    }
}
