package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestClimber extends SubsystemBase{
    private WPI_TalonSRX climbSRX = new WPI_TalonSRX(10);

    public TestClimber(){
        climbSRX.setNeutralMode(NeutralMode.Brake);
        climbSRX.setSafetyEnabled(false);



       
    }

    public void runClimberUp(){
        climbSRX.set(ControlMode.PercentOutput, 0.3);
    }
    public void runClimberDown(){
        climbSRX.set(ControlMode.PercentOutput, -1);

    }
    public void stopClimb(){
        climbSRX.set(ControlMode.Disabled, 1);  }
    
}
