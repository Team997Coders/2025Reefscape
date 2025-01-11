package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Algae extends SubsystemBase{
    
    private final SparkMax spinnyMotor;
    private final SparkMaxConfig spinnyMotorConfig;

    public Algae() {
        spinnyMotor = new SparkMax(Constants.Algae.spinnyMotorID, MotorType.kBrushless);
        spinnyMotorConfig = new SparkMaxConfig();

        spinnyMotorConfig.smartCurrentLimit(Constants.Algae.spinnyMotorConfig);
    }

    public void SpinMotor(double speed){
        spinnyMotor.set(speed);
    }

}


