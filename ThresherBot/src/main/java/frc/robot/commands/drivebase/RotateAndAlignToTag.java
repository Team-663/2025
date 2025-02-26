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
public class RotateAndAlignToTag extends Command {
   private static final double rotateKP = 0.035;
   private static final double rotateKD = 0.0;
   private static final double rotateDeadband = 0.5;
   private static final double strafeKP = 0.075;
   private static final double strafeKD = 0.00;
   private static final double strafeDeadband = 0.25;
   private final PIDController m_rotatePID = new PIDController(rotateKP, 0.0, rotateKD);
   private final PIDController m_strafePID = new PIDController(strafeKP, 0.0, strafeKD);
   private final String limelightName = "limelight-ps";
   DoubleSupplier m_forwardSpeed;
   double currForwardSpeed = 0.0;
   double currStrafeSpeed = 0.0;
   double currRotateSpeed = 0.0;
   private final SwerveSubsystem m_swerveSubsystem; // Use SwerveSubsystem
   double m_strafeOffset = 0.0;
   boolean m_useRotate = false;

   
   
   public RotateAndAlignToTag(SwerveSubsystem swerveSubsystem, DoubleSupplier forwardSpeed, double strafeOffset, boolean useRotate) 
   {
      m_swerveSubsystem = swerveSubsystem;
      m_forwardSpeed = forwardSpeed;
      m_strafeOffset = strafeOffset;
      m_useRotate = useRotate;
      addRequirements(swerveSubsystem);
      // Use addRequirements() here to declare subsystem dependencies.
   }

   public RotateAndAlignToTag(SwerveSubsystem swerveSubsystem, DoubleSupplier forwardSpeed) 
   {
      this(swerveSubsystem, forwardSpeed, 0.0, false);
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() 
   {
      m_rotatePID.reset();
      m_strafePID.reset();
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute() 
   {
      boolean hasTarget = LimelightHelpers.getTV(limelightName);
      
      if (hasTarget) 
      {
         double tx = LimelightHelpers.getTX(limelightName);
         //double ty = LimelightHelpers.getTY(limelightName);
         if (m_useRotate)
         {
            currRotateSpeed = m_rotatePID.calculate(tx, 0);
         }
         currStrafeSpeed = 0.0;//m_strafePID.calculate(ty, 0);
         currStrafeSpeed = MathUtil.clamp(m_strafePID.calculate(tx, m_strafeOffset), -1.0, 1.0)*Constants.AUTO_MAX_STRAFE_CHASSIS_SPEED;
         currForwardSpeed = MathUtil.clamp(m_forwardSpeed.getAsDouble(), -1.0, 1.0)*Constants.AUTO_MAX_STRAFE_CHASSIS_SPEED;
        
         if (Math.abs(currRotateSpeed) < rotateDeadband) 
         {
            currRotateSpeed = 0;
         }
         if (Math.abs(currStrafeSpeed) < strafeDeadband) 
         {
            currStrafeSpeed = 0;
         }
         m_swerveSubsystem.drive(new ChassisSpeeds(currForwardSpeed, currStrafeSpeed, currRotateSpeed));
      }
      else
      {
         m_swerveSubsystem.drive(new ChassisSpeeds(0, 0, 0));
      }

      SmartDashboard.putNumber("ALIGN: Rotate Speed", currRotateSpeed);
      SmartDashboard.putNumber("ALIGN: Strafe Speed", currStrafeSpeed);
      SmartDashboard.putNumber("ALIGN: Forward Speed", currForwardSpeed);
      SmartDashboard.putNumber("ALIGN: Rotate Err", m_rotatePID.getError());
      SmartDashboard.putNumber("ALIGN: Strafe Err", m_strafePID.getError());
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
      return false;
   }
}
