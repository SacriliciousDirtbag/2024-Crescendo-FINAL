package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import java.lang.annotation.Target;

import org.ejml.data.ZMatrix;

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
import pabeles.concurrency.IntOperatorTask.Max;
import frc.robot.Constants;

import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;

import frc.lib.util.CANSparkFlexUtil;




public class feederSubsystem extends SubsystemBase {
    
    //FEEDER MOVEMENT
    public CANSparkFlex m_bottomShootMotor;
    public CANSparkFlex m_topShootMotor;
    public fState fstate;

    private double FeederSpinSpeed;


    //FLYWHEEL MOVEMENT
    public PWMSparkMax m_leftIndexMotor;
    public PWMSparkMax m_rightIndexMotor;

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
    double toFloat;
    double toIntake;
    double toTrap;
    double toFar;//Arbitrary value based on distance, shoots
    double toNear;//Arbitrary value based on distance, shoots
    double MAX;
    double toClimb;
    double TARGET; //target angle

    double M_UP;
    double M_DOWN;

    boolean OVERRIDE = false;


    public feederSubsystem(){

        //FEEDER SPINNER
        m_bottomShootMotor = new CANSparkFlex(Constants.feederSubsystem.leftMotorID, MotorType.kBrushless); //Fixed, Had to Reconfigure Motor 21
        m_topShootMotor = new CANSparkFlex(Constants.feederSubsystem.rightMotorID, MotorType.kBrushless);
        m_bottomShootMotor.setIdleMode(IdleMode.kCoast);
        m_topShootMotor.setIdleMode(IdleMode.kCoast);

        //FLYWHEEL SPINNER
        m_leftIndexMotor = new PWMSparkMax(Constants.shooterSystem.LeftFlyWheelID);
        m_rightIndexMotor = new PWMSparkMax(Constants.shooterSystem.RightFlyWheelID);
        m_leftIndexMotor.setInverted(true);
        m_rightIndexMotor.setInverted(true);


        //ARM MOVEMENT
        m_RightAimingMotor = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.RightAimID, MotorType.kBrushless);
        m_LeftAimingMotor = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.LeftAimID, MotorType.kBrushless);

        m_RightAimingMotor.setIdleMode(IdleMode.kBrake);
        m_LeftAimingMotor.setIdleMode(IdleMode.kBrake);

        a_Encoder = new DutyCycleEncoder(frc.robot.Constants.feederSubsystem.feederEncoderID); //PWM Channel
        
        double ffP = 0.0125; //was 0.05, 0.01
        double ffI = 0;
        double ffD = 0;
        aPID = new PIDController(ffP, ffI, ffD);

        aFeedforward = new ArmFeedforward(0, 0.05, 0); //TODO: Tune Feeder Feedforward

        


        //ARM SETPOINTS
        MIN = 56; //20
        toHome = 56;
        toFloat = 60;
        toIntake = MIN+20; //TODO: calibrate Feeder ARM Setpoints
        toTrap = 0; 
        toFar = 70;
        toNear = 54;
        toClimb = 169.7;
        MAX = 169.7; //was 105.35
        //CANBUS USAGE CONSTRAINTS
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_LeftAimingMotor, CANSparkFlexUtil.Usage.kPositionOnly);
        CANSparkFlexUtil.setCANSparkFlexBusUsage(m_RightAimingMotor, CANSparkFlexUtil.Usage.kPositionOnly);

        // CANSparkMaxUtil.setCANSparkMaxBusUsage(m_leftFlyMotor, Usage.kVelocityOnly);
        // CANSparkMaxUtil.setCANSparkMaxBusUsage(m_rightFlyMotor, Usage.kVelocityOnly);


        //Wheels
        m_bottomShootMotor.setInverted(false);
        m_topShootMotor.setInverted(false);

        //Aim Motor
        m_leftIndexMotor.setInverted(false);

        //Arms
        m_LeftAimingMotor.setInverted(false);
        m_RightAimingMotor.setInverted(true);

        setASetPoint(toHome); //init position

        goAimWheelState(fState.STOP);
        goIndexWheelState(frc.robot.State.sState.STOP, 0);
        fstate = frc.robot.State.fState.STOP;
        sState = frc.robot.State.sState.STOP;

    }

    private double aPos() {
        return a_Encoder.getAbsolutePosition() * 360;
    }

     @Override
    public void periodic(){
        // //AIM
        // m_bottomShootMotor.set(FeederSpinSpeed);
        // m_topShootMotor.set(FeederSpinSpeed);

        //FLY
        // m_leftIndexMotor.set(FlywheelSpinSpeed); //0.5
        // m_rightIndexMotor.set(FlywheelSpinSpeed); //-0.5, Reverse Polarity
        
        //ARM
        aPV = aPos();

        var isDisabled = false;
        
        double aOutput = -aPID.calculate(aPV, aSetPoint);

        
        

        // if(OVERRIDE == true){
        //     if(aState == frc.robot.State.aState.M_UP){
        //         m_LeftAimingMotor.set(0.1);
        //         m_RightAimingMotor.set(0.1);

        //     }

        //     if(aState == frc.robot.State.aState.M_DOWN){
        //         m_LeftAimingMotor.set(-0.1);
        //         m_RightAimingMotor.set(-0.1);

        //     }

        //     if(aState == frc.robot.State.aState.IDLE){
        //         m_LeftAimingMotor.set(0.01);
        //         m_RightAimingMotor.set(0.01);

        //     }
        // } else {
        //     //if within threshold
        //     if(aPV < MIN && aPV > MAX){
        //     m_LeftAimingMotor.set(0);
        //     m_RightAimingMotor.set(0);
        //     }
        //     m_LeftAimingMotor.set(aOutput);
        //     m_RightAimingMotor.set(aOutput);
        // }

        //If desired setpoint is within MIN/MAX
         if(aSetPoint >= MIN && aSetPoint <= MAX){
            m_LeftAimingMotor.set(aOutput); //was aOutput
            m_RightAimingMotor.set(aOutput); //was aOutput
            isDisabled = false;
        }else{
             m_LeftAimingMotor.set(0);
             m_RightAimingMotor.set(0);
             isDisabled = true;
        }

        SmartDashboard.putNumber("Feeder Arm Pos", aPV); //Measured in Degrees
        SmartDashboard.putNumber("Feeder Encoder DIO#", a_Encoder.getSourceChannel());

        SmartDashboard.putNumber("Feeder Fly Speed", FeederSpinSpeed);
        SmartDashboard.putString("Flywheel State", sState.name());

        SmartDashboard.putNumber("A Output", aOutput);

        SmartDashboard.putNumber("A Setpoint", getASetPoint());
        SmartDashboard.putBoolean("Is Disabled", isDisabled);

        SmartDashboard.putNumber("Target Angle:", TARGET);
        SmartDashboard.putBoolean("Feeder isOverride", OVERRIDE);

    }


    //FLYWHEEL SPIN STATE
    public void goIndexWheelState(sState state, double speed){
        if(state == frc.robot.State.sState.OUT)
        {
            FlywheelSpinSpeed = speed; //0.4 BLACK wheels
            m_leftIndexMotor.set(FlywheelSpinSpeed); //0.2
            m_rightIndexMotor.set(FlywheelSpinSpeed); //-0.2, Reverse Polarity
            sState = frc.robot.State.sState.OUT;
        }

        if(state == frc.robot.State.sState.IN)
        {
            FlywheelSpinSpeed = -speed; //was 0.2
            m_leftIndexMotor.set(FlywheelSpinSpeed); //was +
            m_rightIndexMotor.set(FlywheelSpinSpeed); //-0.5, Reverse Polarity
            sState = frc.robot.State.sState.IN;
        }

        if(state == frc.robot.State.sState.STOP)
        {
            FlywheelSpinSpeed = 0;
            m_leftIndexMotor.set(FlywheelSpinSpeed); //0.5
            m_rightIndexMotor.set(FlywheelSpinSpeed); //-0.5, Reverse Polarity
            sState = frc.robot.State.sState.STOP;
        }
    }
    

    //AIM SPIN STATE
    public void goAimWheelState(fState state){ //shooter state
        if(state == frc.robot.State.fState.OUT)
        {
            FeederSpinSpeed = 0.75; //Blue wheels
            m_bottomShootMotor.set(FeederSpinSpeed);
            m_topShootMotor.set(FeederSpinSpeed);
            
            m_bottomShootMotor.setInverted(false);
            m_topShootMotor.setInverted(false);
            fstate = frc.robot.State.fState.OUT;
        }

        if(state == frc.robot.State.fState.IN)
        {
            FeederSpinSpeed = -0.2;
            m_bottomShootMotor.set(FeederSpinSpeed);
            m_topShootMotor.set(0);
            
            m_bottomShootMotor.setInverted(false);
            m_topShootMotor.setInverted(false);
            fstate = frc.robot.State.fState.IN;
        }

        if(state == frc.robot.State.fState.STOP)
        {
            m_bottomShootMotor.set(0);
            m_topShootMotor.set(0);
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
     public void goFeederArmState(aState state){ 
        if (state == frc.robot.State.aState.INTAKE_POS) {
            OVERRIDE = false;
            setASetPoint(toIntake);
            aState = frc.robot.State.aState.INTAKE_POS;
             
        }
        if (state == frc.robot.State.aState.TRAP_POS) {
            OVERRIDE = false;
            //she P on my I till i D
            setASetPoint(toTrap);
            aState = frc.robot.State.aState.TRAP_POS;
        }
        if (state == frc.robot.State.aState.AIM_FAR) {
            OVERRIDE = false;
            //she P on my I till i D
            setASetPoint(toFar);
            aState = frc.robot.State.aState.AIM_FAR;
        }
        if (state == frc.robot.State.aState.AIM_NEAR) {
            OVERRIDE = false;
            //she P on my I till i D
            setASetPoint(toNear);
            aState = frc.robot.State.aState.AIM_NEAR;
        }
        if (state == frc.robot.State.aState.HOME) {
            OVERRIDE = false;
            //she P on my I till i D
            setASetPoint(MIN);
            aState = frc.robot.State.aState.HOME;
        }
        if (state == frc.robot.State.aState.CLIMB) {
            OVERRIDE = false;
            //she P on my I till i D
            setASetPoint(toClimb);
            aState = frc.robot.State.aState.CLIMB;
        }
        if (state == frc.robot.State.aState.FLOAT) {
            OVERRIDE = false;
            //she P on my I till i D
            setASetPoint(toFloat);
            aState = frc.robot.State.aState.FLOAT;
        }
       
        
        if (state == frc.robot.State.aState.M_UP) {
            OVERRIDE = true;
            aState = frc.robot.State.aState.M_UP;
        }
         if (state == frc.robot.State.aState.M_DOWN) {
            OVERRIDE = true;
            aState = frc.robot.State.aState.M_DOWN;
        }
         if (state == frc.robot.State.aState.M_IDLE) {
            OVERRIDE = true;
            aState = frc.robot.State.aState.M_IDLE;
        }
    }

    public void stopWheels(){
        goAimWheelState(frc.robot.State.fState.STOP);
    }
   }
