package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.State.iState;

import frc.lib.util.CANSparkFlexUtil;
import frc.lib.util.CANSparkFlexUtil.Usage;;


public class intakeSubsystem extends SubsystemBase {
    public CANSparkFlex m_wheelMotor;
    public CANSparkFlex m_floorMotor;
    public PWMSparkMax m_frontMotor;
    public iState Istate;

    private double spinSpeed = 0;
    private double spinCurrentLimit;

    private ArrayList<Double> wheelVoltage = new ArrayList<>(10);

    

    public intakeSubsystem(){
        m_wheelMotor = new CANSparkFlex(Constants.IntakeSystem.IntakeWheel.wheelMotorID, MotorType.kBrushless);
        m_wheelMotor.setIdleMode(IdleMode.kBrake);

        m_floorMotor = new CANSparkFlex(Constants.IntakeSystem.IntakeWheel.floorMotorID, MotorType.kBrushless); //tennisgrip

        m_frontMotor = new PWMSparkMax(Constants.IntakeSystem.IntakeWheel.frontMotorID); //tennisgrip

        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_wheelMotor, Usage.kVelocityOnly);
        Istate = frc.robot.State.iState.STOP;

        m_wheelMotor.setInverted(false);
        //used to be false before prac match
        m_frontMotor.setInverted(false); //was true
        m_floorMotor.setInverted(false); //was true


        double currentVoltage = m_wheelMotor.getBusVoltage();
        for(int i = 0; i < wheelVoltage.size(); i++){
            wheelVoltage.set(i, currentVoltage);
        }

        goIntakeWheelState(iState.STOP);
    }


    @Override
    public void periodic(){
        wheelVoltage.add(m_wheelMotor.getBusVoltage());
        wheelVoltage.remove(0);

        SmartDashboard.putNumber("Average Intake Voltage", getAverage());
    }

    public double getAverage(){
        double total = 0;
        for(double i : wheelVoltage){
            total += i;
        }
        return total;
    }


    //INTAKE SPIN
    public void goIntakeWheelState(iState state){
        if(state == frc.robot.State.iState.IN){
            m_wheelMotor.set(-0.55); 
            m_frontMotor.set(-0.75); //was 0.25
            m_floorMotor.set(0.75);

            Istate = iState.IN;
        }

        if(state == frc.robot.State.iState.OUT){
            m_wheelMotor.set(0.55);
            m_frontMotor.set(-0.50);
            m_floorMotor.set(0.50);

            Istate = iState.OUT;
        }

        if(state == frc.robot.State.iState.STOP){
            m_wheelMotor.set(0);
            m_frontMotor.set(0);
            m_floorMotor.set(0);

            Istate = iState.STOP;
        }

        if(state == frc.robot.State.iState.AMP_IN){
            m_wheelMotor.set(0.25); 
            m_frontMotor.set(0.75); //was 0.25
            m_floorMotor.set(0.75);

            Istate = iState.AMP_IN;
        }


    }

}
