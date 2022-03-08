package frc.robot.commands;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Translation2d;
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
   // Create a voltage constraint to ensure we don't accelerate too fast

   public FollowPath(Drivetrain drive, Trajectory selectedPath){
       this.drivetrain = drive;
       this.path = selectedPath;
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
            RamseteCommand ramseteCommand = new RamseteCommand(
                path,
                drivetrain::getPose,
                new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
                drivetrain.m_Kinematics,
                drivetrain::autoSpeeds,
                drivetrain);}


    

   
   public void intialize(){
       drivetrain.m_odometry.resetPosition(path.getInitialPose(), path.getInitialPose().getRotation());
   }
   public void execute(){
   
     // Create config for trajectory
     
    
}
}

