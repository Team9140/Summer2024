package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Telemetry {
    private final CommandSwerveDrivetrain drivetrain;

    public Telemetry(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public void updateTelemetry() {
        ChassisSpeeds speeds = drivetrain.getCurrentRobotChassisSpeeds();
        SmartDashboard.putNumber("Drivetrain/Velocity_X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Drivetrain/Velocity_Y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Drivetrain/Angular_Velocity", speeds.omegaRadiansPerSecond);

        Pose2d pose = drivetrain.getPose();
        SmartDashboard.putNumber("Drivetrain/Pose_X", pose.getX());
        SmartDashboard.putNumber("Drivetrain/Pose_Y", pose.getY());
        SmartDashboard.putNumber("Drivetrain/Pose_Rotation", pose.getRotation().getDegrees());

        var moduleStates = drivetrain.getState().ModuleStates;
        if (moduleStates != null) {
            for (int i = 0; i < moduleStates.length; i++) {
                SmartDashboard.putNumber("Drivetrain/Module_" + i + "_Angle", moduleStates[i].angle.getDegrees());
                SmartDashboard.putNumber("Drivetrain/Module_" + i + "_Velocity", moduleStates[i].speedMetersPerSecond);
            }
        }
    }
}