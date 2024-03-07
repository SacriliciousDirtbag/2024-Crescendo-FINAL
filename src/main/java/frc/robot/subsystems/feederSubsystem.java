package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.State.aState;
import frc.robot.State.fState;
import frc.robot.State.sState;
import frc.robot.Constants;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

import frc.lib.util.CANSparkFlexUtil;




public class feederSubsystem extends SubsystemBase {
    
    //FEEDER MOVEMENT
    public CANSparkFlex m_LeftFeederMotor;
    public CANSparkFlex m_RightFeederMotor;
    public fState fstate;

    private double FeederSpinSpeed;


    //FLYWHEEL MOVEMENT
    public PWMSparkMax m_leftFlyMotor;
    public PWMSparkMax m_rightFlyMotor;

    private double FlywheelSpinSpeed;

    //ARM MOVEMENT
    public CANSparkFlex m_RightAimingMotor;
    public CANSparkFlex m_LeftAimingMotor;
    public sState sState; 
    public aState aState;

    public DutyCycleEncoder a_Encoder;

    private double armSpeed;

    private PIDController aPID;
    private ArmFeedforward aFeedforward;

    private double aPV; //curr position
    private double aSetPoint; //destination we want to go to

    //POSE PARAMETERS
    double MIN;
    double toHome;
    double toIntake;
    double toTrap;
    double toFar;//Arbitrary value based on distance, shoots
    double toNear;//Arbitrary value based on distance, shoots
    double MAX;
    

    public feederSubsystem(){

        //FEEDER SPINNER
        m_LeftFeederMotor = new CANSparkFlex(Constants.feederSubsystem.leftMotorID, MotorType.kBrushless); //Fixed, Had to Reconfigure Motor 21
        m_RightFeederMotor = new CANSparkFlex(Constants.feederSubsystem.rightMotorID, MotorType.kBrushless);

        //FLYWHEEL SPINNER
        m_leftFlyMotor = new PWMSparkMax(Constants.shooterSystem.LeftFlyWheelID);
        m_rightFlyMotor = new PWMSparkMax(Constants.shooterSystem.RightFlyWheelID);


        //ARM MOVEMENT
        m_RightAimingMotor = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.RightAimID, MotorType.kBrushless);
        m_LeftAimingMotor = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.LeftAimID, MotorType.kBrushless);

        m_RightAimingMotor.setIdleMode(IdleMode.kCoast);
        m_LeftAimingMotor.setIdleMode(IdleMode.kCoast);

        a_Encoder = new DutyCycleEncoder(frc.robot.Constants.feederSubsystem.feederEncoderID); //PWM Channel
        
        double ffP = 0.05;
        double ffI = 0;
        double ffD = 0;
        aPID = new PIDController(ffP, ffI, ffD);

        aFeedforward = new ArmFeedforward(0, 0, 0); //TODO: Tune Feeder Feedforward

        


        //ARM SETPOINTS
        MIN = 9.131346228365;
        toIntake = 0; //TODO: calibrate Feeder ARM Setpoints
        toTrap = 0; 
        toFar = 0;
        toNear = 0;
        MAX = 105.35;

        //CANBUS USAGE CONSTRAINTS
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_LeftAimingMotor, CANSparkFlexUtil.Usage.kPositionOnly);
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_RightAimingMotor, CANSparkFlexUtil.Usage.kPositionOnly);

        // CANSparkMaxUtil.setCANSparkMaxBusUsage(m_leftFlyMotor, Usage.kVelocityOnly);
        // CANSparkMaxUtil.setCANSparkMaxBusUsage(m_rightFlyMotor, Usage.kVelocityOnly);


        m_LeftFeederMotor.setInverted(true);
        m_RightFeederMotor.setInverted(true);

        m_leftFlyMotor.setInverted(true);

        fstate = frc.robot.State.fState.STOP;
        sState = frc.robot.State.sState.STOP;

        setASetPoint(10);
    }

    private double aPos() {
        return a_Encoder.getAbsolutePosition() * 360;
    }

     @Override
    public void periodic(){
        //SPINNER
        m_LeftFeederMotor.set(FeederSpinSpeed);
        m_RightFeederMotor.set(FeederSpinSpeed);

        //FLY
        m_leftFlyMotor.set(FlywheelSpinSpeed); //0.5
        m_rightFlyMotor.set(FlywheelSpinSpeed); //-0.5, Reverse Polarity
        
        //ARM
        aPV = aPos();
        
        double aOutput = -aPID.calculate(aPV, aSetPoint);
        // m_RightAimingMotor.set(-aOutput);
        // m_LeftAimingMotor.set(aOutput);

        SmartDashboard.putNumber("Feeder Arm Encoder Rot:", aPos());

        SmartDashboard.putNumber("Feeder Arm Pos", aPV); //Measured in Degrees
        SmartDashboard.putNumber("Feeder Encoder DIO#", a_Encoder.getSourceChannel());

        SmartDashboard.putNumber("Feeder Fly Speed", FeederSpinSpeed);
        SmartDashboard.putString("Feeder State", sState.name());

        SmartDashboard.putNumber("A Setpoint", aSetPoint);

    }


    //FLYWHEEL SPIN STATE
    public void gosState(sState state){
        if(state == frc.robot.State.sState.OUT)
        {
            FlywheelSpinSpeed = 0.2; //0.4 BLACK wheels
            sState = frc.robot.State.sState.OUT;
        }

        if(state == frc.robot.State.sState.IN)
        {
            FlywheelSpinSpeed = -0.2;
            sState = frc.robot.State.sState.IN;
        }

        if(state == frc.robot.State.sState.STOP)
        {
            FlywheelSpinSpeed = 0;
            sState = frc.robot.State.sState.STOP;
        }
    }
    

    //AIM SPIN STATE
    public void goFstate(fState state){ //shooter state
        if(state == frc.robot.State.fState.OUT)
        {
            FeederSpinSpeed = 0.75; //Blue wheels
            fstate = frc.robot.State.fState.OUT;
            
        }

        if(state == frc.robot.State.fState.IN)
        {
            FeederSpinSpeed = -0.75;
            fstate = frc.robot.State.fState.IN;
        }

        if(state == frc.robot.State.fState.STOP)
        {
            FeederSpinSpeed = 0;
            fstate = frc.robot.State.fState.STOP;

            
        }
    }


    //ARM SET SETPOINT
    public void setASetPoint(double setpoint){
        aSetPoint = setpoint;
    }

    //ARM SET SETPOINT
    public double getASetPoint(){
        return aSetPoint;
    }

    //ARM MOVEMENT STATE
     public void goAState(aState state){ 
        if (state == frc.robot.State.aState.INTAKE_POS) {
            setASetPoint(toIntake);
            aState = frc.robot.State.aState.INTAKE_POS;
             
        }
        if (state == frc.robot.State.aState.TRAP_POS) {
            //she P on my I till i D
            setASetPoint(toTrap);
            aState = frc.robot.State.aState.TRAP_POS;
        }
        if (state == frc.robot.State.aState.AIM_FAR) {
            //she P on my I till i D
            setASetPoint(toFar);
            aState = frc.robot.State.aState.AIM_FAR;
        }
        if (state == frc.robot.State.aState.AIM_NEAR) {
            //she P on my I till i D
            setASetPoint(toNear);
            aState = frc.robot.State.aState.AIM_NEAR;
        }
        if (state == frc.robot.State.aState.HOME) {
            //she P on my I till i D
            setASetPoint(MIN);
            aState = frc.robot.State.aState.HOME;
        }
    }

    public void stopWheels(){
        goFstate(frc.robot.State.fState.STOP);
    }
   }
