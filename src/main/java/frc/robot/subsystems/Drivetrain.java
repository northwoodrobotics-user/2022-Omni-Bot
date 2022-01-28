// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  public CANSparkMax leftfront = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax rightfront = new CANSparkMax(0, MotorType.kBrushless);
  public CANSparkMax SlaveRight = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax SlaveLeft = new CANSparkMax(3, MotorType.kBrushless);
  public RelativeEncoder leftDriveEncoder;
  public RelativeEncoder rightDriveEncoder;

  public ShuffleboardTab drivetrain = Shuffleboard.getTab("Drivetrain");



  public DifferentialDrive arcade;
  /** Creates a new ExampleSubsystem. */

  public Drivetrain(){
    SlaveLeft.follow(leftfront);
    SlaveRight.follow(rightfront);
    leftDriveEncoder = leftfront.getEncoder();
    rightDriveEncoder = rightfront.getEncoder();


    arcade = new DifferentialDrive(leftfront, rightfront);
  }

  public void ArcadeDrive(double speed, double rotation){
    arcade.arcadeDrive(speed, rotation);
  }
 

  @Override
  public void periodic() {
    drivetrain.addNumber("LeftRPM", ()-> leftDriveEncoder.getVelocity());
    drivetrain.addNumber("LeftRPM", ()-> rightDriveEncoder.getVelocity());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
