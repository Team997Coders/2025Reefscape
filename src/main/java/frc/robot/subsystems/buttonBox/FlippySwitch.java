package frc.robot.subsystems.buttonBox;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class FlippySwitch extends JoystickButton
{
    public FlippySwitch(GenericHID joystick, int button)
    {
        super(joystick, button);
    }

    public boolean Flipped()
    {
        return this.getAsBoolean();
    }
}
