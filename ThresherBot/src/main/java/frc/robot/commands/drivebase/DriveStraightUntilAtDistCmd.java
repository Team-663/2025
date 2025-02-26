// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivebase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem; // Use the provided SwerveSubsystem
import swervelib.SwerveDrive;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveStraightUntilAtDistCmd extends Command {
   private static final double drivePIDkP = 0.075;
   private static final double drivePIDkD = 0.0;
   private static final double driveDeadband = 0.25;
   private final PIDController drivePID = new PIDController(drivePIDkP, 0.0, drivePIDkD);
   private final SwerveSubsystem m_swerveSubsystem; // Use SwerveSubsystem
   private double m_targetSetpoint = 0.0;
   private boolean m_inReverse = false;
   private double m_error = 0.0;
   private double m_currDriveSpeed = 0.0;
   private boolean m_atSetpoint = false;

   /** Creates a new DriveStraightUntilAtDistCmd. */
   public DriveStraightUntilAtDistCmd(SwerveSubsystem swerveSubsystem, double target, boolean inReverse) 
   {
      m_swerveSubsystem = swerveSubsystem;
      m_targetSetpoint = Math.max(target, Constants.AUTO_LASER_DIST_AT_BUMPERS);
      m_inReverse = inReverse;
      addRequirements(swerveSubsystem);

   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() 
   {
      drivePID.reset();
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() 
   {
      double laserDist = m_swerveSubsystem.getLaserDistanceInches();
      m_error = laserDist - m_targetSetpoint;
      m_atSetpoint = Math.abs(m_error) < driveDeadband; 
      double direction = 1.0;

      if (m_inReverse)
         direction = -1.0;

      if (!m_atSetpoint)
      {
         m_currDriveSpeed = MathUtil.clamp(drivePID.calculate(m_error, 0.0), -1.0, 1.0)*Constants.AUTO_MAX_DRIVE_CHASSIS_SPEED * direction;
      }
      else
      {
         m_currDriveSpeed = 0.0;
      }

      m_swerveSubsystem.drive(new ChassisSpeeds(m_currDriveSpeed, 0.0, 0.0));

      SmartDashboard.putNumber("AUTO: Speed", m_currDriveSpeed);
      SmartDashboard.putNumber("AUTO: Error", m_error);
      SmartDashboard.putNumber("AUTO: Set", m_targetSetpoint);
      SmartDashboard.putBoolean("AUTO: ARRIVED", m_atSetpoint);
      
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) 
   {
      m_swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() 
   {
      return (Math.abs(m_error) < driveDeadband);
   }
}
