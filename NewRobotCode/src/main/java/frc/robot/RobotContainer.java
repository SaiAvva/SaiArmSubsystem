// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.JoystickArmControlCmd;
import frc.robot.commands.MoveArmToPositionCmd;
import frc.robot.commands.TelescopingArmControlCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //Initiallizing the Joystick
  private final ArmSubsystem armSubsystem = ArmSubsystem.getInstance();
  private final CommandXboxController joystickArm = new CommandXboxController(Constants.OperatorConstants.kJoystickPort);
  private final JoystickArmControlCmd joystickArmControlCmd = new JoystickArmControlCmd(armSubsystem, ()->joystickArm.getLeftY());
  private final TelescopingArmControlCmd telescopingArmControlCmd = new TelescopingArmControlCmd(armSubsystem, () -> joystickArm.getRightY());
 

  // The robot's subsystems and commands are defined here...
  public double supplier(){
    return joystickArm.getLeftY();
  }

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // private final JoystickButton moveToPosition1Button = new JoystickButton(joystickArm, 1);
  // private final JoystickButton moveToPosition2Button = new JoystickButton(joystickArm,2);
  // private final JoystickButton moveToPosition3Button = new JoystickButton(joystickArm,3);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Binding the buttons to the commands inside the Constructor

    //When Button 1 is pressed, move the arm to the lower position
    // moveToPosition1Button.onTrue(new MoveArmToPositionCmd(armSubsystem, Constants.ArmConstants.POSITION_LOW));
    // //When Button 2 is pressed, move the arm to the middle position
    // moveToPosition2Button.onTrue(new MoveArmToPositionCmd(armSubsystem, Constants.ArmConstants.POSITION_MID));
    // //When Button 3 is pressed, move the arm to the high position
    // moveToPosition3Button.onTrue(new MoveArmToPositionCmd(armSubsystem, Constants.ArmConstants.POSITION_HIGH));
   configureBindings();
    
  }

  private void configureBindings(){
    armSubsystem.setDefaultCommand(joystickArmControlCmd);

    if(Math.abs(joystickArm.getLeftY()) < 0.05){
    armSubsystem.setBaseSpeed(0);
    }

    if(Math.abs(armSubsystem.getBasePosition()) > 180){
    armSubsystem.setBaseSpeed(0);
    }
    
    joystickArm.a().onTrue(new IntakeCmd(armSubsystem,true));
    joystickArm.a().onFalse(new IntakeCmd(armSubsystem, false));
    
  }

  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
}
