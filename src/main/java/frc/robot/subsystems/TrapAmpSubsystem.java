package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.tState;

public class TrapAmpSubsystem extends SubsystemBase {
    public CANSparkMax m_trapMotor;
    public tState tState;

    private double spinSpeed;
    private double spinCurrentLimit;
    

    public TrapAmpSubsystem(){
        m_trapMotor = new CANSparkMax(0, MotorType.kBrushless);


        tState = frc.robot.State.tState.STOP;
    }


    @Override
    public void periodic(){
        m_trapMotor.setIdleMode(IdleMode.kBrake);
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
}
