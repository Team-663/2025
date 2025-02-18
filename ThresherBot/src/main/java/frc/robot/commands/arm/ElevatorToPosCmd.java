// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.ArmConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToPosCmd extends Command {

   Arm m_arm;
   double m_elevPos;

  /** Creates a new ArmToPosCmd. */
  public ElevatorToPosCmd( Arm arm, double elevPos)
  {
      m_arm = arm;
      m_elevPos = elevPos;
      addRequirements(m_arm);
  }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() 
   {
      m_arm.setElevatorPosition(m_elevPos);
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute()
   {
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) 
   {
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished() 
   {
      boolean isElevDone = m_arm.isElevatorAtPosition(m_elevPos);
      SmartDashboard.putBoolean("ArmCMD: ElevDone", isElevDone);
      return (isElevDone);
   }
}
