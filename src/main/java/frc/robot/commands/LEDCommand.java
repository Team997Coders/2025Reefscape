package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;


public class LEDCommand extends Command{
        private LED m_led;
    

    public LEDCommand(LED led){
        m_led = led;
    }



    @Override 
  public void initialize(){
 
  }

  @Override
  public void execute() {
    m_led.SetColor();
  }

  @Override 
  public void end(boolean interrupted) {
   
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
