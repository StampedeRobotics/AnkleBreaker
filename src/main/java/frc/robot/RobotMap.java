package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;

public class RobotMap {
    public static final XboxController CONTROLLER = new XboxController(0);

    public static final AHRS GYRO = new AHRS(SPI.Port.kMXP);

    static final WPI_TalonFX FRONT_LEFT_ANGLE_MOTOR = new WPI_TalonFX(9);
    static final WPI_TalonFX FRONT_LEFT_DRIVE_MOTOR = new WPI_TalonFX(8);
    static CANCoder FRONT_LEFT_ANGLE_ENCODER = new CANCoder(2);
    static final double FRONT_LEFT_ANGLE_OFFSET = 0;

    static final WPI_TalonFX FRONT_RIGHT_ANGLE_MOTOR = new WPI_TalonFX(11);
    static final WPI_TalonFX FRONT_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(10);
    static CANCoder FRONT_RIGHT_ANGLE_ENCODER = new CANCoder(1);
    static final double FRONT_RIGHT_ANGLE_OFFSET = 0;

    static final WPI_TalonFX BACK_LEFT_ANGLE_MOTOR = new WPI_TalonFX(7);
    static final WPI_TalonFX BACK_LEFT_DRIVE_MOTOR = new WPI_TalonFX(6);
    static CANCoder BACK_LEFT_ANGLE_ENCODER = new CANCoder(3);
    static final double BACK_LEFT_ANGLE_OFFSET = 0;

    static final WPI_TalonFX BACK_RIGHT_ANGLE_MOTOR = new WPI_TalonFX(5);
    static final WPI_TalonFX BACK_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(12);
    static CANCoder BACK_RIGHT_ANGLE_ENCODER = new CANCoder(4);
    static final double BACK_RIGHT_ANGLE_OFFSET = 0;

    public static final SwerveModule FRONT_LEFT = new SwerveModule(1, RobotMap.FRONT_LEFT_DRIVE_MOTOR,
            RobotMap.FRONT_LEFT_ANGLE_MOTOR, RobotMap.FRONT_LEFT_ANGLE_ENCODER, RobotMap.FRONT_LEFT_ANGLE_OFFSET);

    public static final SwerveModule FRONT_RIGHT = new SwerveModule(2, RobotMap.FRONT_RIGHT_DRIVE_MOTOR,
            RobotMap.FRONT_RIGHT_ANGLE_MOTOR, RobotMap.FRONT_RIGHT_ANGLE_ENCODER, RobotMap.FRONT_RIGHT_ANGLE_OFFSET);

    public static final SwerveModule BACK_LEFT = new SwerveModule(3, RobotMap.BACK_LEFT_DRIVE_MOTOR,
            RobotMap.BACK_LEFT_ANGLE_MOTOR, RobotMap.BACK_LEFT_ANGLE_ENCODER, RobotMap.BACK_LEFT_ANGLE_OFFSET);

    public static final SwerveModule BACK_RIGHT = new SwerveModule(4, RobotMap.BACK_RIGHT_DRIVE_MOTOR,
            RobotMap.BACK_RIGHT_ANGLE_MOTOR, RobotMap.BACK_RIGHT_ANGLE_ENCODER, RobotMap.BACK_RIGHT_ANGLE_OFFSET);
}
