package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralOutTake extends Command{
     private Coral m_coral;

    public CoralOutTake(Coral coral){
        m_coral = coral;
    }


  @Override 
  public void initialize(){

  }

  @Override
  public void execute() {
    if (m_coral.BeamBrake2() == true) {
        m_coral.SpinRightMotor(Constants.Coral.motorSpeedOutTake);
        m_coral.SpinLeftMotor(Constants.Coral.motorSpeedOutTake);
    } else if (m_coral.BeamBrake2() == false) {
        m_coral.SpinRightMotor(0);
        m_coral.SpinLeftMotor(0);
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
