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
<<<<<<< HEAD
    private static final int x = 0;
        private final SparkMax leftMotor;
        private final SparkMax rightMotor;
        private final SparkBaseConfig leftConfig;
        private final SparkBaseConfig rightConfig;
        private final DigitalInput firstSensor;
        private final DigitalInput secondSensor;
        public Coral(){
            leftMotor = new SparkMax(Constants.Coral.leftMotorID, MotorType.kBrushless);
            rightMotor = new SparkMax(Constants.Coral.rightMotorID, MotorType.kBrushless);
=======
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    private final SparkBaseConfig leftConfig;
    private final SparkBaseConfig rightConfig;
    public final DigitalInput firstSensor;
    public final DigitalInput secondSensor;
    
    public Coral(){
        leftMotor = new SparkMax(Constants.Coral.leftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.Coral.rightMotorID, MotorType.kBrushless);

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();

        leftConfig.inverted(Constants.Coral.leftMotorInverted);
        rightConfig.inverted(Constants.Coral.rightMotorInverted);

        leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        firstSensor = new DigitalInput(Constants.Coral.coralFirstSensor);
        secondSensor = new DigitalInput(Constants.Coral.coralSecondSensor);
    }

    public Command manualMoveCoralMotorsIntake() {
        return this.runOnce(() -> spinBothMotors(-Constants.Coral.motorSpeedIntake));

    }
    public Command manualMoveCoralMotorsOutake() {
        return this.runOnce(() -> spinBothMotors(Constants.Coral.motorSpeedOutTake));
>>>>>>> dev
    
            leftConfig = new SparkMaxConfig();
            rightConfig = new SparkMaxConfig();
    
            leftConfig.inverted(Constants.Coral.leftMotorInverted);
            rightConfig.inverted(Constants.Coral.rightMotorInverted);
    
            leftMotor.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
            rightMotor.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    
            firstSensor = new DigitalInput(Constants.Coral.coralFirstSensor);
            secondSensor = new DigitalInput(Constants.Coral.coralSecondSensor);
        }
    
        public Command manualMoveCoralMotorsIntake() {
            return this.runOnce(() -> spinBothMotors(-Constants.Coral.motorSpeedIntake));
    
        }
        public Command manualMoveCoralMotorsOutake() {
            return this.runOnce(() -> spinBothMotors(Constants.Coral.motorSpeedOutTake));
        
        }
    
        public Command CoralStop() {
            return this.runOnce(() -> spinBothMotors(0));
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
        public boolean BeamBrake1(int x){
            x = 1;
            return !firstSensor.get();
        }
        public boolean BeamBrake2(int x){
            x = 0;
            return !secondSensor.get();
        }
        private void loggers() {
            SmartDashboard.putBoolean("beam brake 1", BeamBrake1(x));
        SmartDashboard.putBoolean("beam brake 2", BeamBrake2(x));
    }


    @Override
    public void periodic() {
        loggers();
    }
}