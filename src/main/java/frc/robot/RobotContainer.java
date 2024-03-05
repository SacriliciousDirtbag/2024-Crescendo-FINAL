package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;


//import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
//import frc.robot.commands.*;

import frc.robot.commands.feederCmds.feedIn;
import frc.robot.commands.feederCmds.FeedertoIntake;
import frc.robot.commands.feederCmds.feedOut;
import frc.robot.commands.feederCmds.feedStop;
import frc.robot.commands.feederCmds.flyIn;
import frc.robot.commands.feederCmds.flyOut;
import frc.robot.commands.feederCmds.flyStop;
import frc.robot.commands.trapCmds.trapIn;
import frc.robot.commands.trapCmds.trapOut;
import frc.robot.commands.trapCmds.trapStop;
import frc.robot.commands.intakeCmds.intakeIn;
import frc.robot.commands.intakeCmds.intakeOut;
import frc.robot.commands.intakeCmds.intakeStop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.subsystems.TrapAmpSubsystem;
import frc.robot.subsystems.intakeSubsystem;
import frc.robot.subsystems.photonSubsystem;
import swervelib.SwerveDrive;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.*;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));

  // Replace with CommandPS4Controller or CommandJoystick if needed

    /* Controllers */
    private final XboxController driver = new XboxController(0); //Logitech XboxController
    //private final Joystick driver2 = new Joystick(0); //Logitech Extreme3D Pro
    public final Joystick buttonBoard = new Joystick(1);//External Driver
    public final Joystick buttonBoard2 = new Joystick(2); //External Driver 2


    /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;

    SendableChooser<Command> m_Chooser = new SendableChooser<>();

    // XBOX CONTROLLER //
    private final JoystickButton X_BUTTON = new JoystickButton(driver, XboxController.Button.kX.value); //POSITION

    private final JoystickButton Y_BUTTON = new JoystickButton(driver, XboxController.Button.kY.value); //POSITION
    
    private final JoystickButton A_BUTTON = new JoystickButton(driver, XboxController.Button.kA.value); //TODO: Implement Photon As Button
    
    private final JoystickButton B_BUTTON = new JoystickButton(driver, XboxController.Button.kB.value);

    private final JoystickButton RIGHT_TRIGGER = new JoystickButton(driver, XboxController.Axis.kRightTrigger.value); //HOLD

    private final JoystickButton RIGHT_BUMPER = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final JoystickButton LEFT_TRIGGER = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value); //TODO: Remap all Buttons

    private final JoystickButton LEFT_BUMPER = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    //private final JoystickButton spinAmp = new JoystickButton(driver, XboxController.Button.kY.value);
    
    

    //BUTTON BOARD
    //private final JoystickButton zeroGyro = new JoystickButton(driver, 4); //TODO: Implement ZerGyro As Button
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value);
   

    //private final JoystickButton safeButton = new JoystickButton(buttonBoard, 12); //Reset Button - use later for Assist Controller
     

    /* Subsystems & Commands */
    public final TrapAmpSubsystem s_TrapAmpSubsystem = new TrapAmpSubsystem();
    public final feederSubsystem s_feederSubsystem = new feederSubsystem();
    public final intakeSubsystem s_IntakeSubsystem = new intakeSubsystem();

    public final trapIn c_trapIn = new trapIn(s_TrapAmpSubsystem);
    public final trapOut c_trapOut = new trapOut(s_TrapAmpSubsystem);
    public final trapStop c_trapStop = new trapStop(s_TrapAmpSubsystem);

    public final intakeIn c_IntakeIn = new intakeIn(s_IntakeSubsystem, s_feederSubsystem);
    public final intakeOut c_IntakeOut = new intakeOut(s_IntakeSubsystem);
    public final intakeStop c_IntakeStop = new intakeStop(s_IntakeSubsystem);

    public final feedIn c_feedIn = new feedIn(s_feederSubsystem);
    public final feedOut c_FeedOut = new feedOut(s_feederSubsystem);
    public final feedStop c_FeedStop = new feedStop(s_feederSubsystem);

    public final flyIn c_flyIn = new flyIn(s_feederSubsystem);
    public final flyOut c_flyOut = new flyOut(s_feederSubsystem);
    public final flyStop c_FlyStop = new flyStop(s_feederSubsystem);

    public final FeedertoIntake c_feederToIntake = new FeedertoIntake(s_feederSubsystem, s_IntakeSubsystem);

    //PHOTON
    public final photonSubsystem s_PhotonSubsystem = new photonSubsystem(); //TODO: Finish Photon if Possible
  

 //PHOTON COMMAND
    //PIDController phController = new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kPYController, Constants.AutoConstants.kPThetaController);
    

    
    //public final Command s_Pose2dMovement = new Pose2dMovement(s_Swerve, new Translation2d(2,0), 180, true, false);


   
    //public final PhotonSwerve m_photonCommand = new PhotonSwerve(s_PhotonSubsystem, s_Swerve);



    //NEW CHOREO AUTO
    // public final Command s_swerveMovement = new SequentialCommandGroup(
    //     new choreoTest(s_Swerve)
    // );
   



    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driver.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driver.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driver.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driver::getYButtonPressed,
                                                                   driver::getAButtonPressed,
                                                                   driver::getXButtonPressed,
                                                                   driver::getBButtonPressed);
        
        SmartDashboard.putData(m_Chooser);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRightX(),
        () -> driver.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRightX()); //* 0.5

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> driver.getRightX());


        drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

      
      // Configure the button bindings
        configureButtonBindings();

    }
        
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
       // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

      //driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

        /* Driver Buttons */



        //FEEDER SUBSYSTEM
        RIGHT_TRIGGER.onTrue(c_flyIn);
        RIGHT_TRIGGER.onFalse(c_FeedStop);

        LEFT_BUMPER.onTrue(c_flyOut);
        LEFT_BUMPER.onFalse(c_FlyStop);

        B_BUTTON.onTrue(c_feedIn);
        B_BUTTON.onFalse(c_FeedStop);

        // A_BUTTON.onTrue(c_FeedOut);
        // A_BUTTON.onFalse(c_FeedStop);

        //INTAKE SUBSYSTEM
        RIGHT_BUMPER.onTrue(c_IntakeIn);
        RIGHT_BUMPER.onFalse(c_IntakeStop);

        RIGHT_BUMPER.onTrue(c_IntakeOut);
        RIGHT_BUMPER.onFalse(c_IntakeStop);

        //AMP SUBSYSTEM
        Y_BUTTON.onTrue(c_trapIn);
        Y_BUTTON.onFalse(c_trapStop);

        X_BUTTON.onTrue(c_trapOut);
        X_BUTTON.onFalse(c_trapStop);

        // photonToggle.onTrue(m_photonCommand);
        // photonToggle.onFalse(new InstantCommand(() -> s_Swerve.drive(new Translation2d(), 0,false, false)));
        

        // trapToggle.onTrue(new InstantCommand(()-> s_trapIn.initialize()));
        // trapToggle.onFalse(new InstantCommand(()-> s_TrapAmpSubsystem.stopWheels()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

     public Command getAutonomousCommand(){
        return m_Chooser.getSelected();
     }

    public void setDriveMode()
  {
    //drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}