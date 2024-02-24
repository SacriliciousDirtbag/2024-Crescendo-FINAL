package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class driveDistance extends Command {
    double setX;
    double setY;
    double setAng;

    public driveDistance(double giveX, double giveY, double giveAng, Swerve s_Swerve){
        double x = s_Swerve.getPose().getX();
        double z = s_Swerve.getPose().getRotation().getDegrees();
        
        s_Swerve.drive(new Translation2d(giveX, giveY), giveAng, true, false);
        
                
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }
}