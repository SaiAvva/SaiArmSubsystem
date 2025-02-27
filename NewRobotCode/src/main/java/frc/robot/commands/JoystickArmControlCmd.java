// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class JoystickArmControlCmd extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem armSubsystem;
  

  
  private Supplier<Double> baseSpeedSupplier;
  public JoystickArmControlCmd(ArmSubsystem armSubsystem, Supplier<Double> baseSpeedSupplier) {
    this.armSubsystem = armSubsystem;
    this.baseSpeedSupplier = baseSpeedSupplier;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setBaseSpeed(baseSpeedSupplier.get());


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setBaseSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
