package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase{

    private static Launcher instance;

    private final WPI_TalonSRX topLauncher;
    private final WPI_TalonSRX bottomLauncher;

    private double topLauncherVolts;
    private double bottomLauncherVolts;


    private Launcher(){
        this.topLauncher = new WPI_TalonSRX(Constants.Ports.TOP_LAUNCHER);
        this.bottomLauncher = new WPI_TalonSRX(Constants.Ports.BOTTOM_LAUNCHER);

        this.topLauncher.configContinuousCurrentLimit(Constants.Launcher.TOP_LAUNCHER_MAX_CURRENT);
        this.bottomLauncher.configContinuousCurrentLimit(Constants.Launcher.BOTTOM_LAUNCHER_MAX_CURRENT);
    }

    public static Launcher getInstance() {
        return Launcher.instance == null ? Launcher.instance = new Launcher() : Launcher.instance;
    }

    @Override
    public void periodic(){
        this.topLauncher.setVoltage(this.topLauncherVolts);
        this.bottomLauncher.setVoltage(this.bottomLauncherVolts);
    }

    public Command setTopVolts(int volts){
        return this.runOnce(()-> this.topLauncherVolts=volts);
    }

    public Command setBottomVolts(int volts){
        return this.runOnce(()-> this.bottomLauncherVolts=volts);
    }

    public Command prepareLaunch(){
        return this.setTopVolts(Constants.Launcher.TOP_LAUNCHER_VOLTS);
    }

    public Command launch(){
        return this.setBottomVolts(Constants.Launcher.BOTTOM_LAUNCHER_VOLTS);
    }

    public Command intake(){
        return this.setTopVolts(-Constants.Launcher.TOP_LAUNCHER_VOLTS).andThen(this.setBottomVolts(-Constants.Launcher.BOTTOM_LAUNCHER_VOLTS));
    }

    public Command off(){
        return this.setTopVolts(0).andThen(this.setBottomVolts(0));
    }







}