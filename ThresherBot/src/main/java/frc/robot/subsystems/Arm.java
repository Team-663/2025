// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.commands.arm.ArmToPosCmd;
import frc.robot.commands.arm.ElevatorToPosCmd;
import frc.robot.commands.arm.WristHomeLimit;
import frc.robot.commands.arm.WristToPosCmd;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase
{
   // ELEVATOR
   private final SparkMax m_elevatorMaster = new SparkMax(Constants.ELEVATOR_MASTER_CAN_ID, MotorType.kBrushless);
   private final SparkMax m_elevatorSlave = new SparkMax(Constants.ELEVATOR_SLAVE_CAN_ID, MotorType.kBrushless);
   private final SparkClosedLoopController m_elevatorPID = m_elevatorMaster.getClosedLoopController();
   private RelativeEncoder m_elevatorEncoder;

   private final DigitalInput m_elevatorLimitLow = new DigitalInput(ArmConstants.ELEVATOR_LOW_LIMIT_SWITCH_PORT);
   // WRIST
   private final TalonFX m_wrist = new TalonFX(Constants.ARM_WRIST_CAN_ID);
   //private final CANcoder m_wristEncoder = new CANcoder(Constants.ARM_ENCODER_CAN_ID); 
   private final PositionVoltage m_wristVoltage = new PositionVoltage(0);
   private final DigitalInput m_wristLimitLow = new DigitalInput(ArmConstants.WRIST_LOW_LIMIT_SWITCH_PORT);
   //private final DutyCycleEncoder m_wristRevEnc = new DutyCycleEncoder(0);
   //private final LaserCan m_laser = new LaserCan(Constants.LASER_CAN_A_ID);

   private ShuffleboardTab tab = Shuffleboard.getTab("Arm");
   private ComplexWidget dbArmSubsystemEntry = tab.add(this);

   private GenericEntry dbLaserEntry = tab.add("Laser Distance", 0).getEntry();
   private GenericEntry dbWristAngle = tab.add("Wrist Angle", 0).getEntry();
   private GenericEntry dbWristSetpoint = tab.add("Wrist Set", 0).getEntry();
   private GenericEntry dbWristCurrent = tab.add("Wrist Amps", 0).getEntry();
   private GenericEntry dbWristMotorV = tab.add("Wrist Volts", 0).getEntry();
   
   private GenericEntry dbElvOutput = tab.add("Elv Output", 0).getEntry();
   private GenericEntry dbElvEncRaw = tab.add("Elv Enc Raw", 0).getEntry();
   private GenericEntry dbElvEncVel = tab.add("Elv Enc Vel", 0).getEntry();
   private GenericEntry dbElvSetpoint = tab.add("Elv Setpoint", 0).getEntry();
   private GenericEntry dbArmScoreLevel = tab.add("Arm Level", 0).getEntry();


   private final StatusSignal<Boolean> f_fusedSensorOutOfSync = m_wrist.getFault_FusedSensorOutOfSync(false);
   private final StatusSignal<Boolean> sf_fusedSensorOutOfSync = m_wrist.getStickyFault_FusedSensorOutOfSync(false);
   private final StatusSignal<Boolean> f_remoteSensorInvalid = m_wrist.getFault_RemoteSensorDataInvalid(false);
   private final StatusSignal<Boolean> sf_remoteSensorInvalid = m_wrist.getStickyFault_RemoteSensorDataInvalid(false);

   private final StatusSignal<Angle> fx_pos = m_wrist.getPosition(false);
   private final StatusSignal<AngularVelocity> fx_vel = m_wrist.getVelocity(false);
   //private final StatusSignal<Angle> cc_pos = m_wristEncoder.getPosition(false);
   //private final StatusSignal<AngularVelocity> cc_vel = m_wristEncoder.getVelocity(false);
   private final StatusSignal<Angle> fx_rotorPos = m_wrist.getRotorPosition(false);

   private final DutyCycleOut m_dutyCycleControl = new DutyCycleOut(0);

   private int printCount = 0;

   private double m_wristSetpoint = 0.0;
   private double m_wristError = 0.0;
   private double m_elevatorSetpoint = 0.0;
   private double m_elevatorError = 0.0;
   private boolean m_elevatorWristInterlockEnabled = true;
   private boolean m_elevatorMovementBlocked = false;
   private boolean m_wristMovementBlocked = false;
   private boolean m_wristLimitLowState = false;
   private boolean m_elevatorUsePid = false;
   private int m_armScoreLevel = 0;

   private enum CoralLevel {
      NONE,L1,L2,L3,L4

    }

   public Arm()
   {
      ConfigureWrist();
      ConfigureElevator();
   }
   // GUIDES: https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/closed-loop-guide.html
   // https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/PositionClosedLoop/src/main/java/frc/robot/Robot.java

   @Override
   public void periodic()
   {
      // This method will be called once per scheduler run
      //ExecuteWristPIDPeriodic();
   
      // TODO: we don't need this
      UpdateSmartDashboard();

   }

   private void updateElevatorPIDSettings(double p, double i, double d)
   {
      //m_elevatorPID.SetD(1.0);

      SparkMaxConfig pidConfig = new SparkMaxConfig();
      pidConfig.closedLoop.pidf(  p,
                                 i,
                                 d,
                                 ArmConstants.ELEVATOR_PID_FF);
      //pidConfig.apply()
   }

   public void setWristHere()
   {
      m_wristSetpoint = m_wrist.getPosition().getValueAsDouble();
      setWristPosition(m_wristSetpoint);
   }
   public double getWristPosition()
   {
      return m_wrist.getPosition().getValueAsDouble();
   }

   public boolean isSafeToMoveWristDown()
   {
      return (m_elevatorEncoder.getPosition() > ArmConstants.ELEVATOR_POS_NEUTRAL);
   }

   public boolean isWristAtLowLimit()
   {
      return !m_wristLimitLow.get();
   }

   public boolean isWristAtPosition(double position)
   {
      double currentPos = m_wrist.getPosition().getValueAsDouble();
      m_wristError = Math.abs(position - currentPos);
      return (m_wristError < ArmConstants.WRIST_ERROR_TOLERANCE);
   }

   public void setWristPosition(double position)
   {
      // Safety check: do not move wrist up unless elevator is up first
      //if (position > ArmConstants.WRIST_POS_ELV_SAFE && m_elevatorEncoder.getPosition() < ArmConstants.ELEVATOR_POS_NEUTRAL)
      //{
      //   m_wristMovementBlocked = true;
      //}
      //else
      {
         m_wristMovementBlocked = false;
         m_wristSetpoint = position;
         m_wrist.setControl(m_wristVoltage.withPosition(m_wristSetpoint));
      }
  
   }

   public void setElevatorHere()
   {
      m_elevatorSetpoint = m_elevatorEncoder.getPosition();
      m_elevatorUsePid = true;
      setElevatorPosition(m_elevatorSetpoint);
   }

   public boolean isElevatorAtPosition(double position)
   {
      double currentPos = m_elevatorEncoder.getPosition();
      m_elevatorError = Math.abs(position - currentPos);
      return m_elevatorError < ArmConstants.ELEVATOR_ERROR_TOLERANCE;
   }

   public void setElevatorPosition(double position)
   {
      m_elevatorUsePid = true;
      // SAFETY CHECK: DO NOT move elevator down UNLESS wrist is down first
      //if (position < ArmConstants.ELEVATOR_POS_NEUTRAL && m_wrist.getPosition().getValueAsDouble() > ArmConstants.WRIST_POS_ELV_SAFE)
      //{
         // We no safety checks while we are debugging this
      //   if (m_elevatorWristInterlockEnabled)
      //      m_elevatorMovementBlocked = true;
      //   else
      //      m_elevatorMovementBlocked = false;
      //}
      //else
      m_elevatorMovementBlocked = false;
      m_elevatorSetpoint = position;
      //m_elevatorError = Math.abs(m_elevatorSetpoint - currentPos);
      // TODO: some kind of offsets here possible?

      // TODO: if you need to apply arbFF do it here, not for elevator i think
      
      m_elevatorPID.setReference(m_elevatorSetpoint, ControlType.kPosition);
      
   }

   private boolean allowElevatorMotion(double speed)
   {
      // TODO: enable eventually
      /*
      if (m_elevatorLimitHigh.get() && speed > 0)
      {
         return false;
      }
      else if (m_elevatorLimitLow.get() && speed < 0)
      {
         return false;
      }
      */
      return true;
   }

   public void setArmSafe()
   {
      m_elevatorUsePid = false;
      m_wristMovementBlocked = true;
      moveElevatorOpenLoop(0);
      moveWristOpenLoop(0);
   }

   public void resetAllArmEncoders()
   {
      m_elevatorEncoder.setPosition(0);
      m_wrist.setPosition(0.0);
   }

   public void resetWristEncoder()
   {
      m_wrist.setPosition(0.0);
   }

   public void moveElevatorOpenLoop(double speed)
   {
      double output = linearDeadband(speed, 0.1);
      output = MathUtil.clamp(output, ArmConstants.ELEVATOR_MAX_OUTPUT_DOWN, ArmConstants.ELEVATOR_MAX_OUTPUT_UP);
      m_elevatorUsePid = false;
      if (allowElevatorMotion(output))
      {
         m_elevatorMaster.set(output);
      }
      else
      {
         m_elevatorMaster.set(0);
      }
      SmartDashboard.putNumber("ELV: openLoopOut", output);
   }

   public void moveWristOpenLoop(double speed)
   {
      double output = linearDeadband(speed, 0.1);
      double wristPos = m_wrist.getPosition().getValueAsDouble();
      double wristDownMaxSpeed = ArmConstants.WRIST_MAX_OUTPUT_DOWN;
      if (wristPos < ArmConstants.WRIST_POS_VERY_SLOW)
         wristDownMaxSpeed = ArmConstants.WRIST_MAX_OUTPUT_DOWN_VERY_SLOW;

      output = MathUtil.clamp(output, wristDownMaxSpeed, ArmConstants.WRIST_MAX_OUTPUT_UP);

      if (isWristAtLowLimit() && output < 0.0)
         output = 0.0;

      m_wrist.setControl(m_dutyCycleControl.withOutput(output));
                                           //.withLimitReverseMotion(m_wristLimitLow.get()));
   }

   public Command armSetScoreLevelCmd(int level)
   {
      return Commands.runOnce(()->{m_armScoreLevel = level;})
                     .withName("SetScoreLevel");
   }

   
   private int getScoreLevel()
   {
      return m_armScoreLevel;
   }

   public Command armScoreCoralCmd()
   {
      return new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
               Map.entry(2, scoreOnL2EndCmd()),
               Map.entry(3, scoreOnL3EndCmd()),
               Map.entry(4, scoreOnL4EndCmd())),
            this::getScoreLevel);
   }

   public Command armPrepCoralCmd()
   {
      return new SelectCommand<>(
            // Maps selector values to commands
            Map.ofEntries(
               Map.entry(2, scoreOnL2PrepCmd()),
               Map.entry(3, scoreOnL3PrepCmd()),
               Map.entry(4, scoreOnL4PrepCmd())),
            this::getScoreLevel);
   }
   /*
   public Command armScoreCmd()
      {
         int level = m_armScoreLevel;
         SmartDashboard.putString("Score Level", "-");
         switch(level)
         {
            case 2:
               SmartDashboard.putString("Score Level", "L2");
               return scoreOnL2EndCmd();
            case 3:
               SmartDashboard.putString("Score Level", "L3");
               return scoreOnL3EndCmd();
            case 4:
            SmartDashboard.putString("Score Level", "L4");
               return scoreOnL4EndCmd();
            default:
               return Commands.none();

         }
      }
         */

   public Command moveElevatorToNeutralCmd()
   {
      return Commands.sequence(
         new FunctionalCommand(()->{m_armScoreLevel = 0;}
                                 ,()->setElevatorPosition(ArmConstants.ELEVATOR_POS_NEUTRAL)
                                 , null
                                 , ()-> isElevatorAtPosition(ArmConstants.ELEVATOR_POS_NEUTRAL)
                                 , this
                                 )
         
      );
   }

   public Command moveWristToNeutralCmd()
   {
      return Commands.sequence(
         new FunctionalCommand(()-> setWristPosition(ArmConstants.WRIST_POS_DOWN)
                                 , null
                                 , null
                                 , ()-> isWristAtPosition(ArmConstants.WRIST_POS_DOWN)
                                 , this
                                 )
         
      );
   }

   public Command moveArmToNeutralCmd()
   {
      return Commands.sequence(
          new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_NEUTRAL)
          ,new WristHomeLimit(this, ArmConstants.WRIST_MAX_OUTPUT_DOWN)
         ,new WristToPosCmd(this, ArmConstants.WRIST_POS_DOWN)
      ).withName("ArmToNeutral");
   }

   public Command armLoadCoralCmd()
   {
      return Commands.sequence(
         moveArmToNeutralCmd()
         ,new WaitCommand(0.2)
         ,new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_DOWN).withTimeout(0.75)
         ,new WaitCommand(0.1)
         ,new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_NEUTRAL)
         ///new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_DOWN)
      ).withName("ArmLoadCoral");
   }

   public Command scoreOnL2PrepCmd()
   {
      return Commands.sequence(
         new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_SCORE_L2)
         ,new WristToPosCmd(this, ArmConstants.WRIST_POS_SCORE_PREP_L2)
         ,armSetScoreLevelCmd(2)
         )
                                 .withName("ScoreOnL2Prep");
   }

   public Command scoreOnL2EndCmd()
   {
      return Commands.sequence(
         new WristToPosCmd(this, ArmConstants.WRIST_POS_SCORE_END_L2)
         ,armSetScoreLevelCmd(2)
      ).withName("ScoreOnL2End");
   }

   public Command scoreOnL3PrepCmd()
   {
      return Commands.sequence(
         new WristToPosCmd(this, ArmConstants.WRIST_POS_SCORE_PREP_L3)
         ,new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_SCORE_L3)
         ,armSetScoreLevelCmd(3)
      ).withName("ScoreOnL3Prep");
   }

   public Command scoreOnL3EndCmd()
   {
      return Commands.sequence(

         new WristToPosCmd(this, ArmConstants.WRIST_POS_SCORE_END_L3)
         ,new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_DOWN)
         
      ).withName("ScoreOnL3End");
   }

   public Command scoreOnL4PrepCmd()
   {
      return Commands.sequence(
         new WristToPosCmd(this, ArmConstants.WRIST_POS_SCORE_PREP_L4)
         ,new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_SCORE_L4)
         ,armSetScoreLevelCmd(4)
      ).withName("ScoreOnL4Prep");
   }

   public Command scoreOnL4EndCmd()
   {
      return Commands.sequence(
         new WristToPosCmd(this, ArmConstants.WRIST_POS_SCORE_END_L4)
         
      ).withName("ScoreOnL4End");
   }
   
   public Command armByXboxCmd(DoubleSupplier elevValue, DoubleSupplier wristValue)
   {
      return run( ()-> {
         moveElevatorOpenLoop(elevValue.getAsDouble());
         moveWristOpenLoop(wristValue.getAsDouble());
      })
               .withName("ArmByXbox");
   }

   public Command wristHomeCmd()
   {
      return Commands.sequence(
         new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_NEUTRAL)
         ,new WristHomeLimit(this, ArmConstants.WRIST_MAX_OUTPUT_DOWN_VERY_SLOW)
      ).withName("WristHome");
   }

   public Command algaeFromLowerCmd()
   {
      return Commands.sequence(
         new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_DOWN)
         ,new WristToPosCmd(this, ArmConstants.WRIST_POS_RIGHT_ANGLE)
      ).withName("AlgaeFromLower");
   }

   public Command algaeFromUpperCmd()
   {
      return Commands.sequence(
         new ElevatorToPosCmd(this, ArmConstants.ELEVATOR_POS_ALGAE_UPPER)
         ,new WristToPosCmd(this, ArmConstants.WRIST_POS_RIGHT_ANGLE)
      ).withName("AlgaeFromUpper");
   }

   public Command autoScoreOnL4Cmd()
   {
      return Commands.sequence(
         scoreOnL4PrepCmd()
         ,new WaitCommand(0.1)
         ,scoreOnL4EndCmd()
      ).withName("AutoScoreOnL4");
   }

   public void armStopIfClosedLoop()
   {
      if (!m_elevatorUsePid)
      {
         moveElevatorOpenLoop(0);
         moveWristOpenLoop(0);
      }
   }
   public Command armStopCmd()
   {
      return run( ()->{
         armStopIfClosedLoop();
                  })
               .withName("ArmStop");
   }

   public Command setElevPositionCmd(double setpoint)
   {
      return new InstantCommand(()->setElevatorPosition(setpoint)).withName("ElevSetPos");
   }

   public void ConfigureElevator()
   {
      m_elevatorSetpoint = 0.0;
      m_elevatorEncoder = m_elevatorMaster.getEncoder();
      SparkMaxConfig masterConfig = new SparkMaxConfig();
      SparkMaxConfig slaveConfig = new SparkMaxConfig();
      
      masterConfig.inverted(true);
      // TODO; do we setup PID here? Need to check REV lib
      masterConfig.closedLoop.pidf(  ArmConstants.ELEVATOR_PID_P,
                                     ArmConstants.ELEVATOR_PID_I,
                                     ArmConstants.ELEVATOR_PID_D,
                                     ArmConstants.ELEVATOR_PID_FF)
                              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                              .positionWrappingEnabled(false)
                              .outputRange(ArmConstants.ELEVATOR_MAX_OUTPUT_DOWN, ArmConstants.ELEVATOR_MAX_OUTPUT_UP);
                              //.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
      masterConfig.encoder.positionConversionFactor(ArmConstants.ELEVATOR_ENC_CONV_FACTOR);
      masterConfig.openLoopRampRate(0.5);
      masterConfig.softLimit.forwardSoftLimit(ArmConstants.ELEVATOR_POS_MAX_HEIGHT);
      masterConfig.softLimit.forwardSoftLimitEnabled(true);

      masterConfig.softLimit.reverseSoftLimit(ArmConstants.ELEVATOR_POS_DOWN);
      masterConfig.softLimit.reverseSoftLimitEnabled(true);

      m_elevatorMaster.configure(masterConfig, SparkBase.ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

      // Slave is just set to follow
      slaveConfig.follow(Constants.ELEVATOR_MASTER_CAN_ID, true);
      m_elevatorSlave.configure(slaveConfig,SparkBase.ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);  

   }

   public void setWristNeutralMode(boolean brake)
   {
      if (brake)
      {
         m_wrist.setNeutralMode(NeutralModeValue.Brake);
      }
      else
      {
         m_wrist.setNeutralMode(NeutralModeValue.Coast);
      }
   }

   public void ConfigureWrist()
   {
      MotorOutputConfigs mconfig = new MotorOutputConfigs();

      mconfig.withInverted(InvertedValue.Clockwise_Positive);
      mconfig.withNeutralMode(NeutralModeValue.Brake);

      TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
      fx_cfg.Feedback.SensorToMechanismRatio = 125.0/360.0;
      fx_cfg.Feedback.RotorToSensorRatio = 1.0;

      fx_cfg.Slot0.kP = ArmConstants.WRIST_PID_P;
      fx_cfg.Slot0.kI = ArmConstants.WRIST_PID_I;
      fx_cfg.Slot0.kD = ArmConstants.WRIST_PID_D;

      fx_cfg.SoftwareLimitSwitch.withForwardSoftLimitThreshold(ArmConstants.WRIST_POS_MAX_ANGLE)
                                 .withReverseSoftLimitThreshold(0.0)
                                 .withForwardSoftLimitEnable(true)
                                 .withReverseSoftLimitEnable(false);


      fx_cfg.Voltage.withPeakForwardVoltage(Volts.of(12 * ArmConstants.WRIST_MAX_OUTPUT_UP))
         .withPeakReverseVoltage(Volts.of(12 * ArmConstants.WRIST_MAX_OUTPUT_DOWN));

      fx_cfg.CurrentLimits.withStatorCurrentLimit(40.0);
      fx_cfg.CurrentLimits.withStatorCurrentLimitEnable(true);

      fx_cfg.withMotorOutput(mconfig);

      StatusCode status = m_wrist.getConfigurator().apply(fx_cfg);
      
      if (!status.isOK())
      {
         System.out.println("Wrist configuration failed: " + status);
      }

 
      // TODO: fix this after testing
      //m_wrist.setPosition(ArmConstants.WRIST_POS_DOWN);
   }

   /*
   public void ConfigureWristCurrentLimit(double amps)
   {
      TalonFXConfiguration fx_cfg = new TalonFXConfiguration();
      fx_cfg.CurrentLimits.withStatorCurrentLimit(amps);
      fx_cfg.CurrentLimits.withStatorCurrentLimitEnable(true);
      StatusCode status = m_wrist.getConfigurator().apply(fx_cfg);
   }
      */

   private void UpdateSmartDashboard()
   {
      SmartDashboard.putData("ARM Subsystem", this);
      SmartDashboard.putNumber("ARM: PID_P", m_elevatorMaster.configAccessor.closedLoop.getP());
      SmartDashboard.putNumber("ARM: PID_I", m_elevatorMaster.configAccessor.closedLoop.getI());
      SmartDashboard.putNumber("ARM: PID_D", m_elevatorMaster.configAccessor.closedLoop.getD());
      SmartDashboard.putBoolean("ARM: PID ENABLED", m_elevatorUsePid);
      SmartDashboard.putBoolean("ARM: WristLimit", isWristAtLowLimit());

      SmartDashboard.putNumber("ARM: Wrist StatorCurrent", m_wrist.getStatorCurrent().getValueAsDouble());
      dbArmScoreLevel.setInteger(m_armScoreLevel);
      // I don't think we need this, the subsystem is sent to the tab now
      
      dbLaserEntry.setDouble(0);
      dbWristAngle.setDouble(m_wrist.getPosition().getValueAsDouble());
      dbWristSetpoint.setDouble(m_wristSetpoint);
      dbWristCurrent.setDouble(m_wrist.getTorqueCurrent().getValueAsDouble());
      dbWristMotorV.setDouble(m_wrist.getMotorVoltage().getValueAsDouble());

      dbElvOutput.setDouble(m_elevatorMaster.getAppliedOutput());
      dbElvEncRaw.setDouble(m_elevatorEncoder.getPosition());
      dbElvEncVel.setDouble(m_elevatorEncoder.getVelocity());
      dbElvSetpoint.setDouble(m_elevatorSetpoint);
   
   }

   private static double linearDeadband(double raw, double deadband)
   {

      if (Math.abs(raw)<deadband) return 0;

      return Math.signum(raw)*(Math.abs(raw)-deadband)/(1-deadband);
   }

}
