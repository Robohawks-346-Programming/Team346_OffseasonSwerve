// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/** An example command that uses an example subsystem. */
public class JoystickDrive extends CommandBase {

  Drivetrain drivetrain;
  DoubleSupplier x,y,theta;

  public JoystickDrive(Drivetrain drivetrain, DoubleSupplier x, DoubleSupplier y, DoubleSupplier theta) {
    this.drivetrain = drivetrain;
    this.x = x;
    this.y = y;
    this.theta = theta;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double doubleX = Math.abs(x.getAsDouble()) < 0.05 ? 0 : x.getAsDouble();
    double doubleY = Math.abs(y.getAsDouble()) < 0.05 ? 0 : y.getAsDouble();
    double doubleTheta = Math.abs(theta.getAsDouble()) < 0.05 ? 0 : theta.getAsDouble();

    double vx = doubleX * Constants.MAX_MOVE_VELOCITY;
    double vy = doubleY * Constants.MAX_MOVE_VELOCITY;
    double omega = doubleTheta * Constants.MAX_TURN_VELOCITY;

    ChassisSpeeds velocity = Constants.IS_FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, drivetrain.getHeading()) 
      : new ChassisSpeeds(vx, vy, omega);

    ChassisSpeeds percent = new ChassisSpeeds(doubleX, doubleY, doubleTheta);

    drivetrain.drive(velocity, percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
