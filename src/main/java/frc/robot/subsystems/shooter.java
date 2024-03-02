// package frc.robot.subsystems;

// import com.revrobotics.CANSparkFlex;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.PWM;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.State.iState;
// import frc.robot.State.sState;
// import frc.lib.util.CANSparkFlexUtil;
// import frc.lib.util.CANSparkFlexUtil.Usage;;


// public class shooter extends SubsystemBase {
//     public CANSparkFlex m_rightShooter;
//     public CANSparkFlex m_leftShooter;
//     public sState state = sState.STOP; 

//     private double spinSpeed = 0;
    

//     public shooter(){
//         m_rightShooter = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.LeftAimID, MotorType.kBrushed);
//         m_leftShooter = new CANSparkFlex(frc.robot.Constants.shooterAimingSystem.RightAimID, MotorType.kBrushed);
//         m_rightShooter.setIdleMode(IdleMode.kCoast);
//         m_leftShooter.setIdleMode(IdleMode.kCoast);
        
//     }


//     @Override
//     public void periodic(){
//         m_rightShooter.set(spinSpeed);        
//         m_rightShooter.set(spinSpeed);        


//     }

//     //INTAKE SPIN
//     public void goIstate(sState state){
//         if(state == frc.robot.State.sState.IN){
//             spinSpeed = -0.75;
//         }

//         if(state == frc.robot.State.sState.OUT){
//             spinSpeed = 0.75;
            
//         }

//         if(state == frc.robot.State.sState.STOP){
//             spinSpeed = 0;

//         }


//     }

// }
