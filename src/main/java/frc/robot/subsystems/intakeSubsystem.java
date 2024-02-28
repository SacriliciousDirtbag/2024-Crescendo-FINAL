package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.State.iState;

import frc.lib.util.CANSparkFlexUtil;
import frc.lib.util.CANSparkFlexUtil.Usage;;


public class intakeSubsystem extends SubsystemBase {
    public CANSparkFlex m_wheelMotor;
    public iState Istate;

    private double spinSpeed;
    private double spinCurrentLimit;
    

    public intakeSubsystem(){
        m_wheelMotor = new CANSparkFlex(Constants.IntakeSystem.IntakeWheel.wheelMotorID, MotorType.kBrushless);
        m_wheelMotor.setIdleMode(IdleMode.kBrake);


        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_wheelMotor, Usage.kVelocityOnly);
        Istate = frc.robot.State.iState.STOP;
    }


    @Override
    public void periodic(){
        m_wheelMotor.set(spinSpeed);

    }

    public double IPos(){
        return m_wheelMotor.getEncoder().getPosition();
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
