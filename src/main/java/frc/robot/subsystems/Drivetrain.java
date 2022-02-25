// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.SPI;

//import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {

  public CANSparkMax leftfront = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax rightfront = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax SlaveRight = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax SlaveLeft = new CANSparkMax(3, MotorType.kBrushless);
  public SparkMaxPIDController leftController;
  public SparkMaxPIDController rightController;

  public RelativeEncoder leftDriveEncoder;
  public RelativeEncoder rightDriveEncoder;

  public ShuffleboardTab drivetrain = Shuffleboard.getTab("Drivetrain");

  public final DifferentialDriveKinematics m_Kinematics = new DifferentialDriveKinematics(
    DriveConstants.TrackWidth
  );

  public final Gyro gyro = new ADXRS450_Gyro();
  //public final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public final DifferentialDriveOdometry m_odometry;




  public DifferentialDrive arcade;
  /** Creates a new ExampleSubsystem. */

  public Drivetrain(){
    SlaveLeft.follow(leftfront);
    SlaveRight.follow(rightfront);
    leftDriveEncoder = leftfront.getEncoder();
    rightDriveEncoder = rightfront.getEncoder();

    leftfront.setInverted(true);
    leftController = leftfront.getPIDController();
    rightController = rightfront.getPIDController();
    

    leftController.setP(Constants.DriveConstants.DriveKp);
    
    leftController.setI(Constants.DriveConstants.Driveki);
    
    leftController.setD(Constants.DriveConstants.DriveKd);
    leftController.setFF(DriveConstants.DriveFF);

    leftController.setIZone(DriveConstants.DriveIZone);
    leftController.setOutputRange(-1, 1);


    rightController.setP(Constants.DriveConstants.DriveKp);
    
    rightController.setI(Constants.DriveConstants.Driveki);
    
    rightController.setD(Constants.DriveConstants.DriveKd);

    rightController.setFF(DriveConstants.DriveFF);

    rightController.setIZone(DriveConstants.DriveIZone);
    rightController.setOutputRange(-1, 1);
    m_odometry = new DifferentialDriveOdometry(getGyro());


    leftDriveEncoder.setPositionConversionFactor(2*Math.PI*DriveConstants.WheelRadius/42);
    rightDriveEncoder.setPositionConversionFactor(2*Math.PI*DriveConstants.WheelRadius/42);

    m_odometry.resetPosition(new Pose2d(), getGyro());









    


    arcade = new DifferentialDrive(leftfront, rightfront);
    arcade.setSafetyEnabled(false);
    arcade.setExpiration(0.1);
  }


  public void setSpeeds(DifferentialDriveWheelSpeeds speeds){
    double LeftOutput = speeds.leftMetersPerSecond;

    double RightOutput = speeds.rightMetersPerSecond;
    rightController.setReference(RightOutput, ControlType.kVelocity);

    leftController.setReference(LeftOutput, ControlType.kVelocity);

  }

  public Rotation2d getGyro(){
 return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public void PIDDrive  (double speed, double rotation){
    var wheelSpeeds = m_Kinematics.toWheelSpeeds(new ChassisSpeeds(speed, 0, rotation));
    setSpeeds(wheelSpeeds);
  
}
public void updateOdometry(){
  m_odometry.update(getGyro(), leftDriveEncoder.getPosition()/9, rightDriveEncoder.getPosition()/9);
}
  public void ArcadeDrive(double speed, double rotation){
    arcade.arcadeDrive(speed, rotation);
  }
  public void RunAtPID(double speed){
    double CommandedSpeed = speed*Constants.DriveConstants.MaxDriveRPM;
    rightController.setReference(CommandedSpeed, ControlType.kVelocity);
    
    leftController.setReference(CommandedSpeed, ControlType.kVelocity);
  }


  public  double ShowLeftDriveSpeeds(){
    return leftDriveEncoder.getVelocity()/9;

  }
  
  public  double ShowRightDriveSpeeds(){
    return rightDriveEncoder.getVelocity()/9;

  }
  
 

  @Override
  public void periodic() {
    updateOdometry();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
