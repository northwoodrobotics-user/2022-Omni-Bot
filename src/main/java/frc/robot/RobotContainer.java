// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import ExternalLib.SpectrumLib.controllers.SpectrumXboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.PIDDRIVE;
import frc.robot.commands.PIDRUN;
import frc.robot.commands.TeleDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TestClimber;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Drivetrain teleDrivetrain = new Drivetrain();
  public static TestClimber climber = new TestClimber();
  //private final TeleDrive teleDrive;
  public ShuffleboardTab master = Shuffleboard.getTab("Master");
  public static SpectrumXboxController xbox;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
     xbox = new SpectrumXboxController(0, 0.3, 0.3);

    teleDrivetrain.setDefaultCommand(new PIDDRIVE(teleDrivetrain, ()->xbox.leftStick.getY()*DriveConstants.MaxDriveRPM, ()->xbox.rightStick.getX()*1000));
    //teleDrivetrain.RunAtPID(xbox2.getLeftY()*5000);


    ShowMaster();
    ShowInputs();
    ShowOdometry();

  

    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xbox.aButton.whenHeld(
      new PIDRUN(()-> xbox.leftStick.getY(), teleDrivetrain)
    );
    xbox.yButton.toggleWhenPressed(
      new TeleDrive(teleDrivetrain, ()-> xbox.leftStick.getY(), ()-> xbox.rightStick.getX())
    );
    xbox.leftBumper.whenHeld(
      new ClimbUp(climber)
    );
    xbox.rightBumper.whenHeld(
      new ClimbDown(climber)
    );
    
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public void ShowMaster(){
    master.addNumber("LeftDriveSpeed", ()-> teleDrivetrain.ShowLeftDriveSpeeds());
    
    master.addNumber("RightDriveSpeed", ()-> teleDrivetrain.ShowRightDriveSpeeds());
    master.addBoolean("testBeamBreak", ()->teleDrivetrain.isBeamBreakTriggered());
  }
  public void ShowInputs(){
    master.addNumber("ControllerY", ()-> xbox.leftStick.getY());
    
    master.addNumber("ControllerX", ()-> xbox.rightStick.getX());
    master.addNumber("PidCommand", ()-> xbox.leftStick.getY()*DriveConstants.MaxDriveRPM);
  }
  public void ShowOdometry(){
    master.addNumber("GyroAngle", ()-> teleDrivetrain.getGyro().getDegrees());
    master.addNumber("OdometryY", ()-> teleDrivetrain.m_odometry.getPoseMeters().getY());
    master.addNumber("OdometryX", ()-> teleDrivetrain.m_odometry.getPoseMeters().getX());


  }

}
