package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.aState;
import frc.robot.State.fState;
import frc.robot.State.sState;
import frc.robot.Constants;

public class feederSubsystem extends SubsystemBase {
    public CANSparkFlex m_feederMotor1;
    public CANSparkFlex m_feederMotor2;
    public fState fstate;

    private double spinSpeed;
    private double spinCurrentLimit;

    //ARM MOVEMENT
    public CANSparkFlex m_AimingMotor1;
    public CANSparkFlex m_AimingMotor2;
    public sState sState; 
    public aState aState;

    public DutyCycleEncoder a_Encoder;

    private double armSpeed;

    private PIDController aPID;
    private ArmFeedforward aFeedforward;

    private double aPV; //curr position
    private double aSetPoint; //destination we want to go to

    //POSE PARAMETERS
    double toIntake;
    double toTrap;
    double toAim; //Arbitrary value based on distance, shoots
    

    public feederSubsystem(){
        //SPINNER MOVEMENT
        m_feederMotor1 = new CANSparkFlex(Constants.feederSubsystem.leftMotorID, MotorType.kBrushless); //wierd, errors if is CanSparkFlex, errors if it is CanSparkMax
        m_feederMotor2 = new CANSparkFlex(Constants.feederSubsystem.rightMotorID, MotorType.kBrushless);

        fstate = frc.robot.State.fState.STOP;


        //ARM MOVEMENT
        m_AimingMotor1 = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.m_aim1, MotorType.kBrushless);
        m_AimingMotor2 = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.m_aim2, MotorType.kBrushless);
        m_AimingMotor1.setIdleMode(IdleMode.kCoast);
        m_AimingMotor2.setIdleMode(IdleMode.kCoast);

        a_Encoder = new DutyCycleEncoder(0); //PWM Channel
        
        double ffP = 0;
        double ffI = 0;
        double ffD = 0;
        aPID = new PIDController(ffP, ffI, ffD);

        aFeedforward = new ArmFeedforward(0, 0.2, 0); //-0.15

        sState = frc.robot.State.sState.STOP;


        //ARM SETPOINTS
        toIntake = 0; //TODO: calibrate
        toTrap = 0; //TODO: calibrate
        toAim = 0; //TODO: calibrate
    
    }

    private double aPos() {
        return a_Encoder.getAbsolutePosition() * 360;
    }


    @Override
    public void periodic(){
        //SPINNER
        m_feederMotor1.setIdleMode(IdleMode.kBrake);
        m_feederMotor1.set(spinSpeed);

        m_feederMotor2.setIdleMode(IdleMode.kBrake);
        m_feederMotor2.set(spinSpeed);

        //ARM
        aPV = aPos();
        double aOutput = aPID.calculate(aPV, aSetPoint);
        m_AimingMotor1.set(aOutput);
        m_AimingMotor2.set(aOutput);

        SmartDashboard.putNumber("Arm Encoder Rot:", aPos());

    }

    public double FPos1(){
        return m_feederMotor1.getEncoder().getPosition();
    }

    public double FPos2(){
        return m_feederMotor2.getEncoder().getPosition();
    }


    //FLYWHEEL SPIN STATE
    public void gosState(sState state){
        if(sState == frc.robot.State.sState.OUT)
        {
            spinSpeed = 0.5;
        }

        if(sState == frc.robot.State.sState.IN)
        {
            spinSpeed = -0.5;
        }

        if(sState == frc.robot.State.sState.STOP)
        {
            spinSpeed = 0.5;
        }
    }
    

    //AIM SPIN STATE
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


    //ARM SET SETPOINT
    public void setASetPoint(double setpoint){
        aSetPoint = setpoint;
    }

    //ARM MOVEMENT STATE
     public void goaState(aState state){ 
        if (state == frc.robot.State.aState.INTAKE_POS) {
            setASetPoint(toIntake);
            aState = frc.robot.State.aState.INTAKE_POS;
             
        }
        if (state == frc.robot.State.aState.TRAP_POS) {
            //she P on my I till i D
            setASetPoint(toTrap);
            aState = frc.robot.State.aState.TRAP_POS;
        }
        if (state == frc.robot.State.aState.AIM_POS) {
            //she P on my I till i D
            setASetPoint(toAim);
            aState = frc.robot.State.aState.AIM_POS;
        }
    }

    public void stopWheels(){
        goFstate(frc.robot.State.fState.STOP);
    }
}
