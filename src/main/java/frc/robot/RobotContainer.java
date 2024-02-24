package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;


//import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.trapCmds.trapIn;
import frc.robot.commands.trapCmds.trapOut;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.subsystems.photonSubsystem;
import frc.robot.subsystems.LEDSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final XboxController driver = new XboxController(0); //Logitech XboxController
    private final Joystick driver2 = new Joystick(0); //Logitech Extreme3D Pro
    public final Joystick buttonBoard = new Joystick(1);//External Driver
    public final Joystick buttonBoard2 = new Joystick(2); //External Driver 2


    /* Drive Controls */
    //private final int translationAxis = XboxController.Axis.kLeftY.value;
    //private final int strafeAxis = XboxController.Axis.kLeftX.value;
    //private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    SendableChooser<Command> m_Chooser = new SendableChooser<>();

    // XBOX CONTROLLER //

    //DRIVE
    private final JoystickButton photonToggle = new JoystickButton(driver, XboxController.Button.kA.value); //TODO: Implement As Button
        private final JoystickButton trapToggle = new JoystickButton(driver, XboxController.Button.kB.value);


    
    private final JoystickButton zeroGyro = new JoystickButton(driver, 4); //TODO: Implement As Button
    private final JoystickButton robotCentric = new JoystickButton(driver2, XboxController.Button.kLeftBumper.value);
   
    
    private final JoystickButton safeButton = new JoystickButton(driver2, 12); //Reset Button - use later for Assist Controller
    private final JoystickButton safeButton2 = new JoystickButton(driver2, 12); //Reset Button
    
 

    /* Subsystems & Commands */
    public final Swerve s_Swerve = new Swerve();
    public final LEDSubsystem s_lightSubsystem = new LEDSubsystem();
    public final TrapAmpSubsystem s_TrapAmpSubsystem = new TrapAmpSubsystem();

    //PHOTON
    public final photonSubsystem s_PhotonSubsystem = new photonSubsystem(); //TODO: Finish

    public final Command m_leftCommand = new left(s_Swerve);
    public final Command m_middleCommand = new middle(s_Swerve);    

 //PHOTON COMMAND
    PIDController phController = new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kPYController, Constants.AutoConstants.kPThetaController);
    

    
    public final Command s_Pose2dMovement = new Pose2dMovement(s_Swerve, new Translation2d(2,0), 180, true, false);


   
    public final PhotonSwerve m_photonCommand = new PhotonSwerve(s_PhotonSubsystem, s_Swerve);



    //NEW CHOREO AUTO
    public final Command s_swerveMovement = new SequentialCommandGroup(
        new choreoTest(s_Swerve)
    );

    //Subsystem Commands
    public final trapIn s_trapIn = new trapIn(s_TrapAmpSubsystem);
    public final trapOut s_TrapOut = new trapOut(s_TrapAmpSubsystem);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -(driver.getRawAxis(translationAxis))/2, //setting to the power of 3 makes for smoother decceleration 
                () -> -(driver.getRawAxis(strafeAxis))/2, // was pow(, 3);
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        
        // Configure the button bindings
        configureButtonBindings();

        SmartDashboard.putData(m_Chooser);
    }
        
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        photonToggle.onTrue(m_photonCommand);
        photonToggle.onFalse(new InstantCommand(() -> s_Swerve.drive(new Translation2d(), 0,false, false)));
        //TODO: Change PostCondition

        trapToggle.onTrue(new InstantCommand(()-> s_trapIn.initialize()));
        trapToggle.onFalse(new InstantCommand(()-> s_TrapAmpSubsystem.stopWheels()));



        
   
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

     public Command getAutonomousCommand(){
        return m_Chooser.getSelected();
     }

    public Command getBalanceCommand() {
        // An ExampleCommand will run in autonomous
        return Commands.sequence(
            new RunCommand(() -> s_Swerve.drive(new Translation2d(-2, 0), 0, true, true), s_Swerve).withTimeout(1)
        );
    }

    public Command getTurnCommand(){
        return new autoTurn();
    }
}