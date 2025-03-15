package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{

    private final SparkMax climbMotor;
    
    public Climber() {
        climbMotor = new SparkMax(Constants.Climber.climbMotorID, MotorType.kBrushless);
    }
    
    public void SpinMotor(double Speed) {
        climbMotor.set(Speed);
    }

    public Command climb() {
        return this.run(() -> SpinMotor(Constants.Climber.climbSpeed));

    }

    public Command unclimb() {
        return this.run(() -> SpinMotor(-Constants.Climber.climbSpeed));

    }

    public Command stopClimb() {
        return this.run(() -> SpinMotor(0));

    }
}
