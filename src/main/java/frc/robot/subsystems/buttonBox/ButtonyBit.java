package frc.robot.subsystems.buttonBox;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonyBit extends JoystickButton
{
    public final int id;

    public ButtonyBit(GenericHID joystick, int button)
    {
        super(joystick, button);
        id = button;
    }

    public boolean pressed()
    {
        return this.getAsBoolean();
    }

    public int id()
    {
        return id;
    }
}
