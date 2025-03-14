// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Timer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadCoralCmd extends Command {

   Arm m_arm;

   /** Creates a new ArmToNeutralCmd. */
   public LoadCoralCmd(Arm arm)
   {
      m_arm = arm;
      addRequirements(m_arm);
   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize()
   {
      // TODO: enable pid???
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
      return false;
   }
}
