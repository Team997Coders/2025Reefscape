package frc.robot.subsystems.buttonBox;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.buttonBox.buttonCommands.SetPressed;

public class ButtonBox 
{
    public FlippySwitch autoDrive;
    public FlippySwitch fullAutoCycles;
    public FlippySwitch scoreSide;
    public FlippySwitch spareFlippySwitch;

    private ButtonyBit level1;
    private ButtonyBit level2;
    private ButtonyBit level3;
    private ButtonyBit level4;

    private ButtonyBit side1;
    private ButtonyBit side2;
    private ButtonyBit side3;
    private ButtonyBit side4;
    private ButtonyBit side5;
    private ButtonyBit side6;

    public ButtonyBit leftSource;
    public ButtonyBit rightSource;

    public ButtonyList reefSide;
    public ButtonyList elevatorLevel;

    public ButtonBox(Joystick joystick)
    {
        //TODO: NEED TO SET BUTTON NUMBERS TO BE CORRECT, THEY ARE NOT
        //----------------------------------------------------------
        autoDrive = new FlippySwitch(joystick, 0);
        fullAutoCycles = new FlippySwitch(joystick, 1);
        spareFlippySwitch = new FlippySwitch(joystick, 2);

        level1 = new ButtonyBit(joystick, 3);
        level2 = new ButtonyBit(joystick, 4);
        level3 = new ButtonyBit(joystick, 5);
        level4 = new ButtonyBit(joystick, 6);

        side1 = new ButtonyBit(joystick, 7);
        side2 = new ButtonyBit(joystick, 8);
        side3 = new ButtonyBit(joystick, 9);
        side4 = new ButtonyBit(joystick, 10);
        side5 = new ButtonyBit(joystick, 11);
        side6 = new ButtonyBit(joystick, 12);

        leftSource = new ButtonyBit(joystick, 13);
        rightSource = new ButtonyBit(joystick, 14);

        scoreSide = new FlippySwitch(joystick, 15);
        //-----------------------------------------------------------

        reefSide = new ButtonyList(Arrays.asList(side1, side2, side3, side4, side5, side6));
        elevatorLevel = new ButtonyList(Arrays.asList(level1, level2, level3, level4));

        level1.onTrue(new SetPressed(level1, elevatorLevel));
        level2.onTrue(new SetPressed(level2, elevatorLevel));
        level3.onTrue(new SetPressed(level3, elevatorLevel));
        level4.onTrue(new SetPressed(level4, elevatorLevel));

        side1.onTrue(new SetPressed(side1, reefSide));
        side2.onTrue(new SetPressed(side2, reefSide));
        side3.onTrue(new SetPressed(side3, reefSide));
        side4.onTrue(new SetPressed(side4, reefSide));
        side5.onTrue(new SetPressed(side5, reefSide));
        side6.onTrue(new SetPressed(side6, reefSide));
    }  
}
