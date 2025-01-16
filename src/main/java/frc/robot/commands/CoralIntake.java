package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntake extends Command {

    private Coral m_coral;

    public CoralIntake(Coral coral){
        m_coral = coral;
    }


  @Override 
  public void initialize(){}

  @Override
  public void execute() {
    if (m_coral.BeamBrake1() == true) {
        m_coral.SpinLeftMotor(Constants.Coral.motorSpeedIntake);
        m_coral.SpinRightMotor(Constants.Coral.motorSpeedIntake);
  } else if (m_coral.BeamBrake1() == false) {
        m_coral.SpinLeftMotor(0);
        m_coral.SpinRightMotor(0);
    }
  }
  

  @Override 
  public void end(boolean interrupted) {
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
