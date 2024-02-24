package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class autoTurn extends Command {

    public void exampleTurn(Swerve s_Swerve){

        s_Swerve.drive(new Translation2d(0.5,0), 0, true, true);
                
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }
}