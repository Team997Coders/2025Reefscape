package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    
    private Servo climberFlipper;

    private Servo coralFlipper1;
    private Servo coralFlipper2;

    public Climber() {
        climberFlipper = new Servo(Constants.Climber.climberFlipperID);

        coralFlipper1 = new Servo(Constants.Climber.coralFlipper1ID);
        coralFlipper2 = new Servo(Constants.Climber.coralFlipper2ID);
    }

    public Command climberFlipperUp() {
        return this.runOnce(() -> climberFlipper.setAngle(Constants.Climber.climberFlipperUpAngle));
    }

    public Command climberFlipperDown() {
        return this.runOnce(() -> climberFlipper.setAngle(Constants.Climber.climberFlipperDownAngle));
    }

    public Command coralFlipperDrop() {
        return this.runOnce(() -> coralFlipper1.setAngle(Constants.Climber.coralFlipperDropAngle))
        .alongWith(this.runOnce(() -> coralFlipper2.setAngle(Constants.Climber.coralFlipperDropAngle)));
    }

    public Command coralFlipperHold() {
        return this.runOnce(() -> coralFlipper1.setAngle(Constants.Climber.coralFlipperHoldAngle)).alongWith(this.runOnce(() -> coralFlipper2.setAngle(Constants.Climber.coralFlipperHoldAngle)));
    }


}

