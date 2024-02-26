package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.aState;
import frc.robot.State.sState;
import frc.robot.Constants;

public class shooterSubsystem extends SubsystemBase {
    public CANSparkFlex m_ShooterMotor1;
    public CANSparkFlex m_ShooterMotor2;
    public sState sState; 

    private double spinSpeed;
    

    public shooterSubsystem(){
        m_ShooterMotor1 = new CANSparkFlex(frc.robot.Constants.shooterSystem.m_shoot1, MotorType.kBrushless);
        m_ShooterMotor2 = new CANSparkFlex(frc.robot.Constants.shooterSystem.m_shoot2, MotorType.kBrushless);
        m_ShooterMotor1.setIdleMode(IdleMode.kCoast);
        m_ShooterMotor2.setIdleMode(IdleMode.kCoast);
        sState = frc.robot.State.sState.STOP;
    }


    @Override
    public void periodic(){
        m_ShooterMotor1.set(spinSpeed);
        m_ShooterMotor1.set(spinSpeed);
    }

    public double SPosM1(){
        return m_ShooterMotor1.getEncoder().getPosition();
    }

    public double SPosM2(){
        return m_ShooterMotor2.getEncoder().getPosition();
    }


    public void gosState(sState state){
        if (state == frc.robot.State.sState.OUT) {
            spinSpeed = 0.4;
        }
        
        if (state == frc.robot.State.sState.STOP) {
            spinSpeed = 0; 
        }
    }

    public void goaState(aState state){
        if (state == frc.robot.State.aState.FAR) {
            //APosM1()
        }
        if (state == frc.robot.State.aState.CLOSE) {
            //she P on my I till i D
        }
        if (state == frc.robot.State.aState.INTAKE) {
            //she P on my I till i D
        }
    }

    public void stopWheels(){
        gosState(frc.robot.State.sState.STOP);
    }

    public void startWheels(){
        gosState(frc.robot.State.sState.OUT);
    }
}
