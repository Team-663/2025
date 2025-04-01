// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TranslateSlowlyCmd extends Command {
  private DoubleSupplier m_speed;
  private final SwerveSubsystem m_swerve;
  /** Creates a new TranslateSlowlyCmd. */
  public TranslateSlowlyCmd(SwerveSubsystem swerve, DoubleSupplier speed)
  {
    m_swerve = swerve;
    m_speed = speed;
    addRequirements(swerve);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double ySpeed = MathUtil.clamp(m_speed.getAsDouble(), -1.0, 1.0) * m_swerve.getSwerveDrive().getMaximumChassisVelocity() * Constants.DRIVE_SLOW_TRANSLATE_SCALE;
    ChassisSpeeds speeds = new ChassisSpeeds(0.0, ySpeed, 0.0); // No ySpeed (strafing)
    m_swerve.setChassisSpeeds(speeds); // Use setChassisSpeeds
    SmartDashboard.putNumber("SWERVE: Slow Translate", ySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
