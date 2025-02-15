package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coral extends SubsystemBase{
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final SparkBaseConfig leftConfig;
    private final SparkBaseConfig rightConfig;
    private final DigitalInput beamBrake1;
    private final DigitalInput beamBrake2;
    public Coral(){
        leftMotor = new SparkMax(Constants.Coral.leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Coral.rightMotorID, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        leftConfig.inverted(Constants.Coral.leftMotorInverted);
        rightConfig.inverted(Constants.Coral.rightMotorInverted);

        leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        beamBrake1 = new DigitalInput(Constants.Coral.beamBrake1ID);
        beamBrake2 = new DigitalInput(Constants.Coral.beamBrake2ID);
    }

    public Command manualMoveCoralMotorsIntake() {
        return this.runOnce(() -> spinBothMotors(Constants.Coral.motorSpeedIntake));

    }
    public Command manualMoveCoralMotorsOutake() {
        return this.runOnce(() -> spinBothMotors(Constants.Coral.motorSpeedOutTake));
    
    }
        
    public void spinBothMotors(double speed) {
        SpinLeftMotor(speed);
        SpinRightMotor(speed);
    }

    public void SpinLeftMotor(double speed){
        leftMotor.set(speed);
    }

    public void SpinRightMotor(double speed){
        rightMotor.set(speed);

    }
    public boolean BeamBrake1(){
        return beamBrake1.get();
    }
    public boolean BeamBrake2(){
        return beamBrake2.get();
    }
    private void loggers() {
        SmartDashboard.putBoolean("beam brake 1", BeamBrake1());
        SmartDashboard.putBoolean("beam brake 2", BeamBrake2());
    }

    @Override
    public void periodic() {
        loggers();
    }
}