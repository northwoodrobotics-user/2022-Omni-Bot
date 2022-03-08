package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestClimber;


public class ClimbDown extends CommandBase{

    private TestClimber testClimber;

    public ClimbDown(TestClimber climber){
        this.testClimber = climber;
    }
    @Override
    public void execute(){
        testClimber.runClimberDown();
    }
    @Override
    public void end(boolean interrupted) {
      testClimber.stopClimb();
    }
    


}
