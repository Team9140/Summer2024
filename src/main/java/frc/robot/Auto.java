package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Auto {
    private final CommandSwerveDrivetrain drivetrain;
    private final SendableChooser<Command> autoChooser;
    private final Map<String, Pose2d> pathInitialPoses = new HashMap<>();

    public Auto(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        configurePathPlanner();
        setupPaths();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : drivetrain.getModuleLocations()) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            () -> drivetrain.getState().Pose, 
            drivetrain::seedFieldRelative,
            drivetrain::getCurrentRobotChassisSpeeds,
            (speeds) -> drivetrain.setControl(drivetrain.getAutoRequest().withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(
                new PIDConstants(10, 0, 0),
                new PIDConstants(10, 0, 0),
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius,
                new ReplanningConfig()
            ),
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
            drivetrain // Subsystem for requirements
        );
    }

    private void setupPaths() {
        pathInitialPoses.put("Path1", new Pose2d(/* x, y, rotation */));
    }

    public Command getAutonomousCommand() {
    //     Command selectedAuto = autoChooser.getSelected();
        
    //     if (selectedAuto != null) {
    //         String autoName = selectedAuto.getName();
    //         Pose2d initialPose = pathInitialPoses.get(autoName);

    //         if (initialPose != null) {
    //             return selectedAuto.beforeStarting(() -> drivetrain.seedFieldRelative(initialPose));
    //         }
    //     }

    //     return selectedAuto;
    // }
    PathPlannerPath path = PathPlannerPath.fromPathFile("Path1");
    return AutoBuilder.followPath(path);
    }
}