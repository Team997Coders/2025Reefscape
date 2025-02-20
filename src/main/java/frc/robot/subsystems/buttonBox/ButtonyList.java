package frc.robot.subsystems.buttonBox;

import java.util.List;

import frc.robot.exceptions.noSelectedButton;
import frc.robot.exceptions.notInList;
import frc.robot.exceptions.outOfBounds;

public class ButtonyList
{
    private List<ButtonyBit> buttonyList;
    private int selectedIdx = -1;

    public ButtonyList(List<ButtonyBit> ButtonyList)
    {
        buttonyList = ButtonyList;
    }

    public ButtonyBit selectedBit() throws noSelectedButton
    {
        if (selectedIdx == -1)
        {
            throw new noSelectedButton("Currently no button selected in buttonyList: " + this);
        }
        return buttonyList.get(selectedIdx);
    }

    public void selectBit(int bit) throws outOfBounds
    {
        if (bit > buttonyList.size()-1)
        {
            throw new outOfBounds("Idx out of bounds for buttonyList: " + buttonyList);
        }
        selectedIdx = bit;
    }

    public int getBitIdx(ButtonyBit buttony) throws notInList
    {
        if (buttonyList.contains(buttony))
        {
            throw new notInList("Button not in buttonyList: " + buttony);
        }
        return buttonyList.indexOf(buttony);
    }

    public ButtonyBit getBitAtIdx(int index)
    {
        return buttonyList.get(index);
    }
}
