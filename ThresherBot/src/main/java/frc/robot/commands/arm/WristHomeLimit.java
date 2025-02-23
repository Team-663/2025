// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristHomeLimit extends Command {

   Arm m_arm;
   double m_downSpeed = 0.0;
   boolean m_dontStart = false;

   public WristHomeLimit(Arm arm, double downSpeed)
   {
      m_arm = arm;
      m_downSpeed = downSpeed;
      addRequirements(m_arm);
         
      // Use addRequirements() here to declare subsystem dependencies.
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize()
   {
      m_dontStart = !m_arm.isSafeToMoveWristDown();
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute()
   {
      double wristPos = m_arm.getWristPosition();
      if (!m_arm.isWristAtLowLimit())
      {
         m_arm.moveWristOpenLoop(m_downSpeed);
      }
      else
      {
         m_arm.moveWristOpenLoop(0.0);
         m_arm.resetWristEncoder();
      }
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) 
   {
      m_arm.moveWristOpenLoop(0.0);
      m_arm.resetWristEncoder();
   }

   // Returns true when the command should end.
  @Override
   public boolean isFinished() 
   {
      return (m_arm.isWristAtLowLimit());
   }
}
