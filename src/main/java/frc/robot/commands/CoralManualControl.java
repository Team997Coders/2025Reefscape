package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Coral;

public class CoralManualControl extends Command {
    private Coral coral;
    private XboxController driveController;
     
    public CoralManualControl(Coral coral, XboxController driveController) {
        this.coral = coral;
        
        this.driveController = driveController;

        addRequirements(coral);
    }

    @Override
    public void execute() {
        if (driveController.getAButton())
        {
        coral.spinBothMotors(Constants.Coral.motorSpeedIntake);
        } else if (driveController.getBButton())
        {
            coral.spinBothMotors(Constants.Coral.motorSpeedOutTake);
        } else if (driveController.getXButton())
        {
            coral.spinBothMotors(-Constants.Coral.motorSpeedIntake);
        } else
        {
            coral.spinBothMotors(0);
        }
    }
}