package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;

public class RobotMap {
  public static final XboxController CONTROLLER = new XboxController(0);

  public static final AHRS GYRO = new AHRS(SPI.Port.kMXP);

  // public static final SwerveModule
}