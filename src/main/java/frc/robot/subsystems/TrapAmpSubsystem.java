package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.tState;

public class TrapAmpSubsystem extends SubsystemBase {
    public CANSparkMax m_trapMotor;
    public CANSparkMax m_trapArmMotor1;
    public CANSparkMax m_trapArmMotor2;
    public tState tState;

    private double spinSpeed;
    private double spinCurrentLimit;
    

    public TrapAmpSubsystem(){
        m_trapMotor = new CANSparkMax(frc.robot.Constants.AmpSystem.m_scorer, MotorType.kBrushless);
        m_trapArmMotor1 = new CANSparkMax(frc.robot.Constants.AmpSystem.m_aim1, MotorType.kBrushless);
        m_trapArmMotor2 = new CANSparkMax(frc.robot.Constants.AmpSystem.m_aim2, MotorType.kBrushless);
        m_trapMotor.setIdleMode(IdleMode.kBrake);
        m_trapArmMotor1.setIdleMode(IdleMode.kBrake);
        m_trapArmMotor2.setIdleMode(IdleMode.kBrake);

        tState = frc.robot.State.tState.STOP;
    }


    @Override
    public void periodic(){
        m_trapMotor.setIdleMode(IdleMode.kBrake);
        m_trapArmMotor1.setIdleMode(IdleMode.kBrake);
        m_trapArmMotor2.setIdleMode(IdleMode.kBrake);
        m_trapMotor.set(spinSpeed);

    }

    public double TPos(){
        return m_trapMotor.getEncoder().getPosition();
    }

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

    public void stopWheels(){
        goTState(frc.robot.State.tState.STOP);
    }

    public void inWheels(){
        goTState(frc.robot.State.tState.IN);
    }

    public void outWheels(){
        goTState(frc.robot.State.tState.OUT);
    }
}
