package frc.robot.subsystems.buttonBox;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.buttonBox.buttonCommands.SetPressed;

public class ButtonBox 
{
    public FlippySwitch autoDrive;
    public FlippySwitch fullAutoElevator;
    public FlippySwitch fullAutoCycles;
    public FlippySwitch semiAutoCycles;

    public ButtonyBit go;

    public ButtonyBit leftScore;
    public ButtonyBit rightScore;

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
    public ButtonyList scoreSide;
    public ButtonyList sourceSide;

    public ButtonBox(XboxController joystick)
    {
        //TODO: NEED TO SET BUTTON NUMBERS TO BE CORRECT, THEY ARE NOT
        //----------------------------------------------------------
        autoDrive = new FlippySwitch(joystick, 15);
        fullAutoCycles = new FlippySwitch(joystick, 18);
        fullAutoElevator = new FlippySwitch(joystick, 16);
        semiAutoCycles = new FlippySwitch(joystick, 17);

        level1 = new ButtonyBit(joystick, 7);
        level2 = new ButtonyBit(joystick, 8);
        level3 = new ButtonyBit(joystick, 9);
        level4 = new ButtonyBit(joystick, 10);

        side1 = new ButtonyBit(joystick, 1);
        side2 = new ButtonyBit(joystick, 2);
        side3 = new ButtonyBit(joystick, 3);
        side4 = new ButtonyBit(joystick, 4);
        side5 = new ButtonyBit(joystick, 5);
        side6 = new ButtonyBit(joystick, 6);

        leftSource = new ButtonyBit(joystick, 13);
        rightSource = new ButtonyBit(joystick, 14);

        rightScore = new ButtonyBit(joystick, 12);
        leftScore = new ButtonyBit(joystick, 11);

        go = new ButtonyBit(joystick, 19);

        //-----------------------------------------------------------

        reefSide = new ButtonyList(Arrays.asList(side1, side2, side3, side4, side5, side6));
        elevatorLevel = new ButtonyList(Arrays.asList(level1, level2, level3, level4));
        scoreSide = new ButtonyList(Arrays.asList(rightScore, leftScore));
        sourceSide = new ButtonyList(Arrays.asList(rightSource, leftSource));
    }  
}
