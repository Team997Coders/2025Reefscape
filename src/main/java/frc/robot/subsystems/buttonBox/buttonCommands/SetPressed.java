package frc.robot.subsystems.buttonBox.buttonCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.buttonBox.ButtonyBit;
import frc.robot.subsystems.buttonBox.ButtonyList;

public class SetPressed extends Command {

  private ButtonyBit buttony;
  private ButtonyList listy;
  
  public SetPressed(ButtonyBit buttonyBit, ButtonyList buttonyList) {
    buttony = buttonyBit;
    listy = buttonyList;
  }

  @Override
  public void initialize() {
    try
    {
      listy.selectBit(listy.getBitIdx(buttony));
    } catch(Exception e)
    {
      e.printStackTrace();
    }
    this.cancel();
  }
}
