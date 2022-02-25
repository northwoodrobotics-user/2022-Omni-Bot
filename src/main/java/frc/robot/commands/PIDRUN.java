package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PIDRUN extends CommandBase{
    Drivetrain subsystem;
    DoubleSupplier PIDCOMMAND;

    public PIDRUN(DoubleSupplier supplier, Drivetrain drivetrain){
        this.subsystem = drivetrain;
        this.PIDCOMMAND = supplier;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute(){
        subsystem.RunAtPID(PIDCOMMAND.getAsDouble());
    }  @Override
    public void end(boolean interrupted) {
      subsystem.RunAtPID(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }



    
}
