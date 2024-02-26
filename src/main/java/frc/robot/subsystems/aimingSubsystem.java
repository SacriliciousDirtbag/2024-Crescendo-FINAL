package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.aState;
import frc.robot.State.sState;
import frc.robot.Constants;

public class aimingSubsystem extends SubsystemBase {
    public static CANSparkFlex m_AimingMotor1;
    public CANSparkFlex m_AimingMotor2;
    public sState sState; 

    public DutyCycleEncoder a_Encoder;

    private double spinSpeed;
    

    public aimingSubsystem(){
        m_AimingMotor1 = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.m_aim1, MotorType.kBrushless);
        m_AimingMotor2 = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.m_aim2, MotorType.kBrushless);
        m_AimingMotor1.setIdleMode(IdleMode.kCoast);
        m_AimingMotor2.setIdleMode(IdleMode.kCoast);

        a_Encoder = new DutyCycleEncoder(0); //PWM Channel
        
        double ffP = 0;
        double ffI = 0;
        double ffD = 0;

        sState = frc.robot.State.sState.STOP;
    }


    private double aPos() {
        return a_Encoder.getAbsolutePosition() * 360;
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Encoder Rot:", aPos());
    }

    public static double APosM1(){
        return m_AimingMotor1.getEncoder().getPosition();
    }

    public double APosM2(){
        return m_AimingMotor2.getEncoder().getPosition();
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

    public void Intake(){
        goaState(frc.robot.State.aState.INTAKE);
    }

    public void Far(){
        goaState(frc.robot.State.aState.FAR);
    }

    public void Close(){
        goaState(frc.robot.State.aState.CLOSE);
    }
}

