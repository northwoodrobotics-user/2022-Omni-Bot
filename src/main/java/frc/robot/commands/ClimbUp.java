package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestClimber;

public class ClimbUp extends CommandBase{
    private TestClimber testClimber;

    public ClimbUp(TestClimber climber){
        this.testClimber = climber;
    }
    @Override
    public void execute(){
        testClimber.runClimberUp();
    }
    @Override
    public void end(boolean interrupted) {
      testClimber.stopClimb();
    }
    
}
