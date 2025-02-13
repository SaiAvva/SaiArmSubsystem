// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
 private static ArmSubsystem INSTANCE;

 //Defining Motor
 private final SparkMax baseMotor;
 private final SparkMax armMotor;
 private final SparkMax endEffectorMotor;
 private final SparkMax intakeMotor;


/** Creates a new ExampleSubsystem. */
  public ArmSubsystem(){
    //Initiallizing the Motors
     baseMotor = new SparkMax(Constants.ArmConstants.baseMotorIDPort, MotorType.kBrushless);
     armMotor = new SparkMax(Constants.ArmConstants.armMotorIDPort, MotorType.kBrushless);
     endEffectorMotor = new SparkMax(Constants.ArmConstants.endEffectorMotorIDPort,MotorType.kBrushless);
     intakeMotor = new SparkMax(Constants.ArmConstants.intakeMotorIDPort,MotorType.kBrushless);
     
     //Declaring Robot Motors Configuration Methods
     RobotConfigs();

     //Resetting the encoders
     resetEncoders();

  }

  private void RobotConfigs(){
    SparkMaxConfig baseMotorConfig = new SparkMaxConfig();
    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    SparkMaxConfig endEffectorMotorConfig = new SparkMaxConfig();
    SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

    baseMotorConfig.inverted(false);
    armMotorConfig.inverted(false);
    endEffectorMotorConfig.inverted(false);
    intakeMotorConfig.inverted(false);

    baseMotorConfig.idleMode(IdleMode.kBrake);
    armMotorConfig.idleMode(IdleMode.kBrake);
    endEffectorMotorConfig.idleMode(IdleMode.kBrake);
    intakeMotorConfig.idleMode(IdleMode.kBrake);

    baseMotorConfig.encoder.positionConversionFactor(Constants.ArmConstants.baseGearRatio*360);
    armMotorConfig.encoder.positionConversionFactor(Constants.ArmConstants.armGearRatio*360);
    endEffectorMotorConfig.encoder.positionConversionFactor(Constants.ArmConstants.endEffectorGearRatio*360);
    intakeMotorConfig.encoder.positionConversionFactor(Constants.ArmConstants.intakeMotorGearRatio*360);

    baseMotorConfig.encoder.velocityConversionFactor(Constants.ArmConstants.baseGearRatio*360*60);
    armMotorConfig.encoder.velocityConversionFactor(Constants.ArmConstants.armGearRatio*360*60);
    endEffectorMotorConfig.encoder.velocityConversionFactor(Constants.ArmConstants.endEffectorGearRatio*360*60);
    intakeMotorConfig.encoder.velocityConversionFactor(Constants.ArmConstants.intakeMotorGearRatio*360*60);

    baseMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04,0,0);
    armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04,0,0);
    endEffectorMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04,0,0);
    intakeMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.04,0,0);

    baseMotor.configure(baseMotorConfig,ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
    armMotor.configure(armMotorConfig,ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
    endEffectorMotor.configure(endEffectorMotorConfig,ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);
    intakeMotor.configure(intakeMotorConfig,ResetMode.kNoResetSafeParameters,PersistMode.kPersistParameters);

  }

  //Moving Arm to specific setpoint
  public void setBasePosition(double Position){
  baseMotor.getClosedLoopController().setReference(Position, ControlType.kPosition);
  }

  public void setArmPosition(double Position){
    armMotor.getClosedLoopController().setReference(Position, ControlType.kPosition);
  }

  public void setEndEffectorPosition(double Position){
    endEffectorMotor.getClosedLoopController().setReference(Position, ControlType.kPosition);
  }

  //Getting the current position of the arm
  public double getBasePosition(){
    return baseMotor.getEncoder().getPosition();
  }

  public double getArmMotorPosition(){
    return armMotor.getEncoder().getPosition();
  }

  public double getEndEffectorPosition(){
    return endEffectorMotor.getEncoder().getPosition();
  }

  //Setting base speed of the motors based on Joystick Input
  public void setBaseSpeed(double speed){
  baseMotor.setVoltage(speed); //Setting Direct speed from Joystick Input
  }

  public void setArmSpeed(double speed){
    armMotor.setVoltage(speed);
  }

  public void setEndEffectorSpeed(double speed){
    endEffectorMotor.setVoltage(speed);
  }

  public void setIntakeSpeed(double speed){
    intakeMotor.setVoltage(speed);
  }

  public void 
  
  
  
  stopIntake(double speed){
    intakeMotor.set(speed);
  }

  


  public void resetEncoders(){}

  public static ArmSubsystem getInstance(){
    if(INSTANCE == null){
      INSTANCE = new ArmSubsystem();
    }
    return INSTANCE;
  }
  

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Base Motor Encoder Value", getBasePosition());
    SmartDashboard.putNumber("Telescoping Motor Encoder Value",getArmMotorPosition());
    SmartDashboard.putNumber("End Effector Position", getEndEffectorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}



