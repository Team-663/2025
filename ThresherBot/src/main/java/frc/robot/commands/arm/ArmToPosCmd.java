// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmToPosCmd extends Command {
   Arm m_arm;
   double m_elevatorPos;
   double m_wristPos;
   /** Creates a new ArmToPosCmd. */
   public ArmToPosCmd(Arm arm, double elevatorPos, double wristPos) {
      m_arm = arm;
      m_elevatorPos = elevatorPos;
      m_wristPos = wristPos;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(m_arm);
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize()
   {
   }

   // Called every time the scheduler runs while the command is scheduled.
   @Override
   public void execute()
   {
      m_arm.setElevatorPosition(m_elevatorPos);
      m_arm.setWristPosition(m_wristPos);
   }

   // Called once the command ends or is interrupted.
   @Override
   public void end(boolean interrupted) {
   }

   // Returns true when the command should end.
   @Override
   public boolean isFinished()
   {
      boolean elevAtPos = m_arm.isElevatorAtPosition(m_elevatorPos);
      boolean wristAtPos = m_arm.isWristAtPosition(m_wristPos);
      return elevAtPos && wristAtPos;
   }
}
