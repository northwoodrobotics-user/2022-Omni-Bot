package frc.robot.commands;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class FollowPath extends CommandBase{
   private  Drivetrain drivetrain;
   private Trajectory path; 
   private Timer timer = new Timer();
   private RamseteController ramsete = new RamseteController(); 
   private double LinearRef; 
   private double AngularRef; 
   // Create a voltage constraint to ensure we don't accelerate too fast

   public FollowPath(Drivetrain drive, Trajectory selectedPath, double LinearVelocity, double AngularVelocity){
       this.drivetrain = drive;
       this.path = selectedPath;
       this.LinearRef = LinearVelocity; 
       this.AngularRef = AngularVelocity; 
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kV,
            DriveConstants.kA),
        drivetrain.m_Kinematics,
        10);
        TrajectoryConfig config =
        new TrajectoryConfig(
                DriveConstants.kMaxMetersPerSecond,
                DriveConstants.kMaxAccelMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(drivetrain.m_Kinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
            

   }
    

   @Override
   public void initialize(){
       timer.reset();
       timer.start();
       drivetrain.m_odometry.resetPosition(path.getInitialPose(), path.getInitialPose().getRotation());
   }
   @Override
   public void execute(){
    Trajectory.State reference = path.sample(timer.get());

    ChassisSpeeds speeds = ramsete.calculate(drivetrain.getPose(), reference.poseMeters, LinearRef, AngularRef);
    drivetrain.setSpeeds(drivetrain.m_Kinematics.toWheelSpeeds(speeds));


    
   
     // Create config for trajectory
     
    
}
@Override
public void end(boolean interrupted){
    drivetrain.PIDDrive(0, 0);
}
}

