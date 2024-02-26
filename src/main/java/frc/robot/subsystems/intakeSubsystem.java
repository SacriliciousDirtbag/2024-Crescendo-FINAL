package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.iState;

public class intakeSubsystem extends SubsystemBase {
    public CANSparkMax m_intakeMotor;
    public iState Istate;

    private double spinSpeed;
    private double spinCurrentLimit;
    

    public intakeSubsystem(){
        m_intakeMotor = new CANSparkMax(0, MotorType.kBrushless);


        Istate = frc.robot.State.iState.STOP;
    }


    @Override
    public void periodic(){
        m_intakeMotor.setIdleMode(IdleMode.kBrake);
        m_intakeMotor.set(spinSpeed);

    }

    public double IPos(){
        return m_intakeMotor.getEncoder().getPosition();
    }

    public void goIstate(iState state){
        if(state == frc.robot.State.iState.IN){
            spinSpeed = -0.4;
        }

        if(state == frc.robot.State.iState.OUT){
            spinSpeed = 0.4;
            
        }

        if(state == frc.robot.State.iState.STOP){
            spinSpeed = 0;

        }


    }

    public void stopWheels(){
            goIstate(frc.robot.State.iState.STOP);
        }
}
