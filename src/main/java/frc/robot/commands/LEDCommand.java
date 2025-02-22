package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.LED;


public class LEDCommand extends Command{
      public static int x;
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
    m_led.ItemPickedUp(x);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
