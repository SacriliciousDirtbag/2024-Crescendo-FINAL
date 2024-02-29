package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.State.iState;

import frc.lib.util.CANSparkFlexUtil;
import frc.lib.util.CANSparkFlexUtil.Usage;;


public class intakeSubsystem extends SubsystemBase {
    public CANSparkFlex m_wheelMotor;
    public CANSparkFlex m_floorMotor;
    public PWMSparkMax m_backMotor;
    public iState Istate;

    private double spinSpeed = 0;
    private double spinCurrentLimit;
    

    public intakeSubsystem(){
        m_wheelMotor = new CANSparkFlex(Constants.IntakeSystem.IntakeWheel.wheelMotorID, MotorType.kBrushless);
        m_wheelMotor.setIdleMode(IdleMode.kBrake);

        m_floorMotor = new CANSparkFlex(Constants.IntakeSystem.IntakeWheel.backMotorID, MotorType.kBrushless);

        m_backMotor = new PWMSparkMax(Constants.IntakeSystem.IntakeWheel.frontMotorID);

        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_wheelMotor, Usage.kVelocityOnly);
        Istate = frc.robot.State.iState.STOP;
    }


    @Override
    public void periodic(){
        m_wheelMotor.set(-spinSpeed);
        m_floorMotor.set(spinSpeed);
        m_backMotor.set(spinSpeed);


    }

    public double IPos(){
        return m_wheelMotor.getEncoder().getPosition();
    }

    //INTAKE SPIN
    public void goIstate(iState state){
        if(state == frc.robot.State.iState.IN){
            spinSpeed = -0.2;
        }

        if(state == frc.robot.State.iState.OUT){
            spinSpeed = 0.2;
            
        }

        if(state == frc.robot.State.iState.STOP){
            spinSpeed = 0;

        }


    }

}
