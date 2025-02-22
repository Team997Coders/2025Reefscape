package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.LED;
public class AlgaeCommandIntake extends Command{
    private Algae m_algae;

    public AlgaeCommandIntake(Algae algae){
        m_algae = algae;
    }


    @Override 
  public void initialize(){

  }

  @Override
  public void execute() {
    if (m_algae.getBeamBreakStatus() == true) {
        m_algae.SpinMotor(Constants.Algae.motorSpin);
        
    } else if (m_algae.getBeamBreakStatus() == false){
        m_algae.SpinMotor(0);
    }
  }

  @Override 
  public void end(boolean interrupted) {
   
  }

  @Override
  public boolean isFinished() {    return false;
  }

}