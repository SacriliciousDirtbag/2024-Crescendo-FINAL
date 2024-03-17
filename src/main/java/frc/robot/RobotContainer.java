package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.robot.State.eState;
import frc.robot.autos.*;
//import frc.robot.commands.*;

import frc.robot.commands.feederCmds.feedIn;
import frc.robot.commands.feederCmds.FeederToHome;
import frc.robot.commands.feederCmds.FeedertoIntake;
import frc.robot.commands.feederCmds.feedOut;
import frc.robot.commands.feederCmds.feedStop;
import frc.robot.commands.feederCmds.flyIn;
import frc.robot.commands.feederCmds.flyOut;
import frc.robot.commands.feederCmds.flyOutFast;
import frc.robot.commands.feederCmds.flyStop;
import frc.robot.commands.feederCmds.shootFar;
import frc.robot.commands.feederCmds.shootHome;
import frc.robot.commands.feederCmds.shootNear;
import frc.robot.commands.feederCmds.toClimb;
import frc.robot.commands.trapCmds.EMERGENCY;
import frc.robot.commands.trapCmds.HOLDIT;
import frc.robot.commands.trapCmds.ampHome;
import frc.robot.commands.trapCmds.hang;
import frc.robot.commands.trapCmds.scoreAmp;
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
// import frc.robot.subsystems.photonSubsystem;
import swervelib.SwerveDrive;
import frc.robot.subsystems.feederSubsystem;
import frc.robot.subsystems.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
import frc.robot.subsystems.SwerveSubsystem;
import java.io.File;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
      private double MaxSpeed = Constants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
      private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

      /* Setting up bindings for necessary control of the swerve drive platform */
      private final SwerveSubsystem drivetrain = Constants.DriveTrain; // My drivetrain


      private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                  // driving in open loop
      private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
      private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
      private final Telemetry logger = new Telemetry(MaxSpeed);
      

  // Replace with CommandPS4Controller or CommandJoystick if needed

    /* Controllers */
    private final CommandXboxController driver = new CommandXboxController(0)/*Logitech XboxController*/;
    private final CommandXboxController driver2 = new CommandXboxController(1); 

    //private final Joystick driver2 = new Joystick(0); //Logitech Extreme3D Pro
    // public final Joystick buttonBoard = new Joystick(1);//External Driver
    // public final Joystick buttonBoard2 = new Joystick(2); //External Driver 2


    /* Drive Controls */
    // private final int translationAxis = XboxController.Axis.kLeftY.value;
    // private final int strafeAxis = XboxController.Axis.kLeftX.value;
    // private final int rotationAxis = XboxController.Axis.kRightX.value;

    private final SendableChooser<Command> m_Chooser;
  
    //BUTTON BOARD
    //private final JoystickButton zeroGyro = new JoystickButton(driver, 4); //TODO: Implement ZeroGyro As Button
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kStart.value);
   

    //private final JoystickButton safeButton = new JoystickButton(buttonBoard, 12); //Reset Button - use later for Assist Controller
     

    /* Subsystems & Commands */
    public final TrapAmpSubsystem s_TrapAmpSubsystem = new TrapAmpSubsystem();
    public final feederSubsystem s_feederSubsystem = new feederSubsystem();
    public final intakeSubsystem s_IntakeSubsystem = new intakeSubsystem();

    public final trapIn c_trapWheelIn = new trapIn(s_TrapAmpSubsystem);
    public final trapOut c_trapWheelOut = new trapOut(s_TrapAmpSubsystem);
    public final trapStop c_trapStop = new trapStop(s_TrapAmpSubsystem);

    public final scoreAmp c_scoreAmp = new scoreAmp(s_TrapAmpSubsystem);

    public final ampHome c_ampHome = new ampHome(s_TrapAmpSubsystem);

    public final intakeIn c_IntakeIn = new intakeIn(s_IntakeSubsystem, s_feederSubsystem);
    public final intakeOut c_IntakeOut = new intakeOut(s_IntakeSubsystem);
    public final intakeStop c_IntakeStop = new intakeStop(s_IntakeSubsystem);

    public final feedIn c_RampWheelIn = new feedIn(s_feederSubsystem);
    public final feedOut c_RampWheelOut = new feedOut(s_feederSubsystem);
    public final feedStop c_RampStop = new feedStop(s_feederSubsystem);

    public final toClimb c_ToClimb = new toClimb(s_feederSubsystem);

    public final flyIn c_IndexWheelIn = new flyIn(s_feederSubsystem);
    public final flyOut c_IndexWheelOut = new flyOut(s_feederSubsystem);
    public final flyOutFast c_flyOutFast = new flyOutFast(s_feederSubsystem);
    public final flyStop c_IndexStop = new flyStop(s_feederSubsystem);

    public final hang c_FloatFeeder = new hang(s_feederSubsystem, s_TrapAmpSubsystem);

    public final shootFar c_AimFar = new shootFar(s_feederSubsystem);
    public final shootNear c_AimNear = new shootNear(s_feederSubsystem);
    public final shootHome c_AimHome = new shootHome(s_feederSubsystem);

    public final FeederToHome c_setAimHome = new FeederToHome(s_feederSubsystem);

    public final EMERGENCY c_OhSHit = new EMERGENCY(s_TrapAmpSubsystem, s_feederSubsystem);
    public final HOLDIT c_HOLDIT = new HOLDIT(s_TrapAmpSubsystem, s_feederSubsystem);

    
    private final SlewRateLimiter translationlimiter = new SlewRateLimiter(Constants.acceleration);
    private final SlewRateLimiter rotationlimiter = new SlewRateLimiter(Constants.turnacceleration);
    private final SlewRateLimiter strafelimiter = new SlewRateLimiter(Constants.strafeacceleration);

    public final FeedertoIntake c_feederToIntake = new FeedertoIntake(s_feederSubsystem, s_IntakeSubsystem);


    //AUTOS
    public final Command s_ScoreBlueOne = new SequentialCommandGroup(
      new Blue1NoteAuto(drivetrain, s_feederSubsystem)
     );

    //PHOTON
    // public final photonSubsystem s_PhotonSubsystem = new photonSubsystem(); //TODO: Finish Photon if Possible

    //PathPlannerTrajectory tesTrajectory
  

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
    
    // Register Named Commands
    NamedCommands.registerCommand("pickupUpCommand", c_IntakeIn);
    NamedCommands.registerCommand("aimCommand", c_AimNear);
    NamedCommands.registerCommand("rampCommand", c_RampWheelOut);
    NamedCommands.registerCommand("shootCommand", c_IndexWheelOut);


      // Configure the button bindings
      configureButtonBindings();

      // Build an auto chooser. This will use Commands.none() as the default option.
     m_Chooser = AutoBuilder.buildAutoChooser();
     SequentialCommandGroup auto = new SequentialCommandGroup(//;new PathPlannerAuto("MF Blue Auto 2"));
      s_ScoreBlueOne, new PathPlannerAuto("MF Blue Auto 2"));
     m_Chooser.setDefaultOption("Score Blue Auto", auto);
    //  m_Chooser.addOption("BF Blue Auto 3", new PathPlannerAuto("BF Blue Auto 3"));
    //  m_Chooser.addOption("MF Blue Auto 2", new PathPlannerAuto("MF Blue Auto 2"));
 
     SmartDashboard.putData("Auto Chooser", m_Chooser);

    }
        
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      
      drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
          drivetrain.applyRequest(() -> drive.withVelocityX(translationlimiter.calculate(driver.getLeftY()) * -MaxSpeed)// Drive forward with
                                                                                            // negative Y (forward)
              .withVelocityY(strafelimiter.calculate(driver.getLeftX()) * -MaxSpeed) //Drive left with negative X (left)
              .withRotationalRate(rotationlimiter.calculate(driver.getRightX()) * -MaxAngularRate) // Drive counterclockwise with negative X (left)
          )); 

      //driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
      //Invert Button?
      // driver.b().whileTrue(drivetrain
      //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

      // reset the field-centric heading on Y Button press
      driver.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
      //driver.leftTrigger.onTrue(drivetrain.runOnce(() -> drivetrain));
      // if (Utils.isSimulation()) {
      //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
      // }
      // drivetrain.registerTelemetry(logger::telemeterize);

    //   driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // driver.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));




      //* PRIMARY DRIVER */

      //INTAKE SUBSYSTEM green wheel intake 
        // driver.leftTrigger().onTrue(c_IntakeIn);
        // driver.leftTrigger().onFalse(c_IntakeStop);

        // //test controls will be changed later if needed
        // driver.rightTrigger().onTrue(c_IntakeOut);
        // driver.rightTrigger().onFalse(c_IntakeStop);

        //FEEDER SUBSYSTEM
        // driver.rightTrigger().onTrue(c_feedIn);  //Blue/Orang Wheels 
        // driver.rightTrigger().onFalse(c_FeedStop);

        // driver.leftBumper().onTrue(c_flyOut);
        // driver.leftBumper().onFalse(c_FlyStop);

        // driver.b().onTrue(c_flyIn); //Black Wheels
        // driver.b().onFalse(c_FlyStop);

        // driver.a().onTrue(c_FeedOut);
        // driver.a().onFalse(c_FeedStop);

        // //INTAKE SUBSYSTEM
        // driver.rightBumper().onTrue(c_IntakeOut); //All Intake Wheels
        // driver.rightBumper().onFalse(c_IntakeStop);

        // driver.rightBumper().onTrue(c_IntakeOut);
        // driver.rightBumper().onFalse(c_IntakeStop);

        // //AMP SUBSYSTEM
        // driver.y().onTrue(c_trapIn); //Trap Wheels
        // driver.y().onFalse(c_trapStop);

        // driver.x().onTrue(c_trapOut);
        // driver.x().onFalse(c_trapStop);

        // driver.leftTrigger().onTrue(c_Aimnear);
        // driver.leftTrigger().onFalse(c_Aimhome);


        //* EXTERNAL DRIVER */
        //FEEDER
        driver2.leftBumper().onTrue(c_IndexWheelOut); //Index Wheels Spin 
        driver2.leftBumper().onTrue(c_IntakeOut);
        driver2.leftBumper().onFalse(c_IndexStop); //feeder for shooter 
        driver2.leftBumper().onFalse(c_IntakeStop);
        
        driver2.leftTrigger().onTrue(c_IndexWheelIn);
        driver2.leftTrigger().onTrue(c_RampWheelIn);
        driver2.leftTrigger().onFalse(c_IndexStop);
        driver2.leftTrigger().onFalse(c_RampStop);

        driver2.x().toggleOnTrue(c_RampWheelOut); //Toggle Ramp Wheel (Aim)
        driver2.x().toggleOnFalse(c_RampStop); // shoot wheels 

        driver2.a().onTrue(c_scoreAmp);
        driver2.a().onFalse(c_ampHome);

        // driver.start().onTrue(); //TODO: Hangar Down
        // driver.start().onFalse(); //Maintain Pos
        
        // driver.back().onTrue(); //TODO: Hangar Up
        // driver.back().onFalse(); //Maintain Pos

        //AMPER
        driver2.rightBumper().onTrue(c_trapWheelOut); //tennis grip bars 
        driver2.rightTrigger().onTrue(c_IntakeIn);
        driver2.rightBumper().onFalse(c_trapStop);
        driver2.rightTrigger().onFalse(c_IntakeStop);
        
        driver2.rightTrigger().onTrue(c_trapWheelIn);
        driver2.rightTrigger().onFalse(c_trapStop);

        driver2.pov(0).onTrue(c_AimFar); //Up D-Pad
        driver2.pov(270).onTrue(c_ToClimb); //Left D-Pad
        driver2.pov(180).onTrue(c_AimNear); //Down D-Pad
        driver2.pov(90).toggleOnTrue(c_FloatFeeder); //Right D-Pad
        driver2.pov(-1).onTrue(c_AimHome); //When not pressed

        
        //INTAKE
        // driver2.start().onTrue(c_OhSHit); //Move Down
        // driver2.start().onFalse(c_HOLDIT);


        // driver2.start().onTrue(c_IntakeIn);
        // driver2.start().onFalse(c_IntakeStop);

        // driver2.back().onTrue(c_IntakeOut);
        // driver2.back().onFalse(c_IntakeStop);


        // photonToggle.onTrue(m_photonCommand);
        // photonToggle.onFalse(new InstantCommand(() -> s_Swerve.drive(new Translation2d(), 0,false, false)));
        // driver.a().onTrue(c_SetAimFar);
        // driver.a().onFalse(c_setAimHome);


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

     public Command resetGyroState(){
            return drivetrain.runOnce(() -> drivetrain.seedFieldRelative(drivetrain.getState().Pose));
     }
  
}