package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.fState;
import frc.robot.Constants;

public class feederSubsystem extends SubsystemBase {
    public CANSparkMax m_feederMotor1;
    public CANSparkMax m_feederMotor2;
    public fState fstate;

    private double spinSpeed;
    private double spinCurrentLimit;
    

    public feederSubsystem(){
        m_feederMotor1 = new CANSparkMax(Constants.feederSubsystem.leftMotorID, MotorType.kBrushless);
        m_feederMotor2 = new CANSparkMax(Constants.feederSubsystem.rightMotorID, MotorType.kBrushless);

        fstate = frc.robot.State.fState.STOP;
    }


    @Override
    public void periodic(){
        m_feederMotor1.setIdleMode(IdleMode.kBrake);
        m_feederMotor1.set(spinSpeed);

        m_feederMotor2.setIdleMode(IdleMode.kBrake);
        m_feederMotor2.set(spinSpeed);
    }

    public double FPos1(){
        return m_feederMotor1.getEncoder().getPosition();
    }

    public double FPos2(){
        return m_feederMotor2.getEncoder().getPosition();
    }

    public void goFstate(fState state){
        if(fstate == frc.robot.State.fState.OUT)
        {
            spinSpeed = 0.5;
        }

        if(fstate == frc.robot.State.fState.IN)
        {
            spinSpeed = -0.5;
        }

        if(fstate == frc.robot.State.fState.STOP)
        {
            spinSpeed = 0.5;
        }
    }

    public void stopWheels(){
        goFstate(frc.robot.State.fState.STOP);
    }
}
