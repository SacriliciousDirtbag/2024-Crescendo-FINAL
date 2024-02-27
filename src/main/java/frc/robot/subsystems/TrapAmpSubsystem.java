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

public class TrapAmpSubsystem extends SubsystemBase {
    public CANSparkMax m_trapMotor;
    public CANSparkMax m_trapArmMotor1;
    public CANSparkMax m_trapArmMotor2;
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
        m_trapMotor = new CANSparkMax(frc.robot.Constants.AmpSystem.m_scorer, MotorType.kBrushless);
        m_trapArmMotor1 = new CANSparkMax(frc.robot.Constants.AmpSystem.m_aim1, MotorType.kBrushless);
        m_trapArmMotor2 = new CANSparkMax(frc.robot.Constants.AmpSystem.m_aim2, MotorType.kBrushless);
        
        m_trapMotor.setIdleMode(IdleMode.kBrake);
        m_trapArmMotor1.setIdleMode(IdleMode.kBrake);
        m_trapArmMotor2.setIdleMode(IdleMode.kBrake);

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
        m_trapArmMotor1.set(tOutput);
        m_trapArmMotor2.set(tOutput);

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
