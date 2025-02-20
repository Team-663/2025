package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swervedrive.SwerveSubsystem; // Use the provided SwerveSubsystem
import swervelib.SwerveDrive;

public class CenterOnAprilTag extends Command {

    // Constants (make configurable via SmartDashboard)
    private static final double kP_AIM = 0.035;
    private static final double kI_AIM = 0.0;
    private static final double kD_AIM = 0.0;
    private static final double AIM_DEADBAND = 0.5;

    private static final double kP_RANGE = 0.1;
    private static final double kI_RANGE = 0.0;
    private static final double kD_RANGE = 0.0;
    private static final double RANGE_DEADBAND = 1.0;
    private static final boolean USE_TY_FOR_RANGE = true;
    private static final double TARGET_AREA = 50; // if using ta
    private static final double DESIRED_AREA = 100; // if using ta

    private final PIDController m_aimPID = new PIDController(kP_AIM, kI_AIM, kD_AIM);
    private final PIDController m_rangePID = new PIDController(kP_RANGE, kI_RANGE, kD_RANGE);

    private final SwerveSubsystem m_swerveSubsystem; // Use SwerveSubsystem
    private final SwerveDrive m_swerveDrive;

    public CenterOnAprilTag (SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        this.m_swerveDrive = swerveSubsystem.getSwerveDrive();
        addRequirements(m_swerveSubsystem); // Declare subsystem dependency

        // Set PID controller setpoints
        m_aimPID.setSetpoint(0);
        m_rangePID.setSetpoint(0);

        // Initialize SmartDashboard values
        SmartDashboard.putNumber("Aim kP", kP_AIM);
        SmartDashboard.putNumber("Aim kI", kI_AIM);
        SmartDashboard.putNumber("Aim kD", kD_AIM);
        SmartDashboard.putNumber("Aim Deadband", AIM_DEADBAND);
        SmartDashboard.putNumber("Range kP", kP_RANGE);
        SmartDashboard.putNumber("Range kI", kI_RANGE);
        SmartDashboard.putNumber("Range kD", kD_RANGE);
        SmartDashboard.putNumber("Range Deadband", RANGE_DEADBAND);
        SmartDashboard.putBoolean("Use TY for Range", USE_TY_FOR_RANGE);
        SmartDashboard.putNumber("Target Area", TARGET_AREA);
        SmartDashboard.putNumber("Desired Area", DESIRED_AREA);
    }

    @Override
    public void initialize() {
        m_aimPID.reset();
        m_rangePID.reset();
    }

    @Override
    public void execute() {
        String limelightName = "limelight-ps"; // configured in limelight pipeline (web interface)
        // Get tuning values from SmartDashboard
        double kP_aim = SmartDashboard.getNumber("Aim kP", kP_AIM);
        double kI_aim = SmartDashboard.getNumber("Aim kI", kI_AIM);
        double kD_aim = SmartDashboard.getNumber("Aim kD", kD_AIM);
        double aimDeadband = SmartDashboard.getNumber("Aim Deadband", AIM_DEADBAND);

        double kP_range = SmartDashboard.getNumber("Range kP", kP_RANGE);
        double kI_range = SmartDashboard.getNumber("Range kI", kI_RANGE);
        double kD_range = SmartDashboard.getNumber("Range kD", kD_RANGE);
        double rangeDeadband = SmartDashboard.getNumber("Range Deadband", RANGE_DEADBAND);
        boolean useTY = SmartDashboard.getBoolean("Use TY for Range", USE_TY_FOR_RANGE);
        double targetArea = SmartDashboard.getNumber("Target Area", TARGET_AREA); //For ta
        double desiredArea = SmartDashboard.getNumber("Desired Area", DESIRED_AREA); //For ta

        // Update PID controllers
        m_aimPID.setPID(kP_aim, kI_aim, kD_aim);
        m_rangePID.setPID(kP_range, kI_range, kD_range);

        if (LimelightHelpers.getTV(limelightName)) {
            // Aiming Logic
            double tx = LimelightHelpers.getTX(limelightName);
            if (Math.abs(tx) < aimDeadband) {
                tx = 0;
            }
            double rot = m_aimPID.calculate(tx);
            rot = MathUtil.clamp(rot, -1.0, 1.0) * m_swerveDrive.getMaximumChassisAngularVelocity(); //Max Angular Speed


            // Ranging Logic
            double error;
            if (useTY) {
                error = LimelightHelpers.getTY(limelightName);
            } else {
                error = LimelightHelpers.getTA(limelightName) - desiredArea; // Use desiredArea
                targetArea = desiredArea;
            }
            if (Math.abs(error) < rangeDeadband) {
                error = 0;
            }
            double xSpeed = m_rangePID.calculate(error);
            xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0) * m_swerveDrive.getMaximumChassisVelocity();  // Max Speed

            // Drive the Robot (using ChassisSpeeds for SwerveSubsystem)
            ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, 0, rot); // No ySpeed (strafing)
            m_swerveSubsystem.setChassisSpeeds(speeds); // Use setChassisSpeeds

        } else {
            // No target: Stop the robot
            m_swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Run until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    }
}
