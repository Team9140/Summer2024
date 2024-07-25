package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Launcher extends SubsystemBase{

    private static Launcher instance;

    private final WPI_TalonSRX topLauncher;
    private final TalonFX bottomLauncher;

    private double topLauncherVolts;
    private final VoltageOut bottomLauncherController;



    private Launcher(){

        this.topLauncher = new WPI_TalonSRX(Constants.Ports.TOP_LAUNCHER);
        this.bottomLauncher =  new TalonFX(Constants.Ports.BOTTOM_LAUNCHER);

        CurrentLimitsConfigs launcherCurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(Constants.Launcher.BOTTOM_LAUNCHER_MAX_CURRENT).withStatorCurrentLimitEnable(true);
        TalonFXConfiguration launcherConfiguration = new TalonFXConfiguration().withCurrentLimits(launcherCurrentLimits);

        this.bottomLauncher.getConfigurator().apply(launcherConfiguration.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
        this.bottomLauncherController = new VoltageOut(0.0).withEnableFOC(true).withUpdateFreqHz(1 / Constants.LOOP_INTERVAL);

        this.topLauncher.configContinuousCurrentLimit(Constants.Launcher.TOP_LAUNCHER_MAX_CURRENT);
    }

    public static Launcher getInstance() {
        return Launcher.instance == null ? Launcher.instance = new Launcher() : Launcher.instance;
    }

    @Override
    public void periodic(){
        this.topLauncher.setVoltage(this.topLauncherVolts);
        this.bottomLauncher.setControl(this.bottomLauncherController);

    }

    public Command setTopVolts(double volts){
        return this.runOnce(()-> this.topLauncherVolts=volts);
    }

    public Command setBottomVolts(double volts){
        return this.runOnce(()-> this.bottomLauncherController.withOutput(volts));
    }

    public Command prepareLaunch(){
        return this.setTopVolts(Constants.Launcher.TOP_LAUNCHER_VOLTS);
    }

    public Command launch(){
        return this.setBottomVolts(Constants.Launcher.BOTTOM_LAUNCHER_VOLTS);
    }

    public Command intake(){
        return this.setTopVolts(-Constants.Launcher.INTAKE_VOLTS).andThen(this.setBottomVolts(-Constants.Launcher.INTAKE_VOLTS));
    }

    public Command amp(){
        return this.setTopVolts(Constants.Launcher.AMP_VOLTS_TOP).andThen(this.setBottomVolts(Constants.Launcher.AMP_VOLTS_BOTTOM));
    }

    public Command off(){
        return this.setTopVolts(0).andThen(this.setBottomVolts(0));
    }







}