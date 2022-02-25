package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Drivetrain;

public class PIDDRIVE extends CommandBase{

    Drivetrain subsystem;
    DoubleSupplier Speed;
    DoubleSupplier Rotation;

    public PIDDRIVE(Drivetrain drivetrain, DoubleSupplier speed, DoubleSupplier rot){
        this.subsystem = drivetrain;
        this.Speed = speed;
        this.Rotation = rot;
        addRequirements(drivetrain);

    }
    public void initialize(){

    }
    public void execute(){
        subsystem.PIDDrive(Speed.getAsDouble(), Rotation.getAsDouble());

    }
    
}
