package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;

public class RobotMap {
        public static final XboxController CONTROLLER = new XboxController(0);

        public static final AHRS GYRO = new AHRS(SPI.Port.kMXP);

        static final double FRONT_LEFT_ANGLE_OFFSET = 0;
        static final double FRONT_RIGHT_ANGLE_OFFSET = 0;
        static final double BACK_LEFT_ANGLE_OFFSET = 0;
        static final double BACK_RIGHT_ANGLE_OFFSET = 0;

        public static final SwerveModule FRONT_LEFT = new SwerveModule(1, new WPI_TalonFX(8), new WPI_TalonFX(9),
                        new CANCoder(2), RobotMap.FRONT_LEFT_ANGLE_OFFSET);

        public static final SwerveModule FRONT_RIGHT = new SwerveModule(2, new WPI_TalonFX(10), new WPI_TalonFX(11),
                        new CANCoder(1), RobotMap.FRONT_RIGHT_ANGLE_OFFSET);

        public static final SwerveModule BACK_LEFT = new SwerveModule(3, new WPI_TalonFX(6), new WPI_TalonFX(7),
                        new CANCoder(3), RobotMap.BACK_LEFT_ANGLE_OFFSET);

        public static final SwerveModule BACK_RIGHT = new SwerveModule(4, new WPI_TalonFX(12), new WPI_TalonFX(5),
                        new CANCoder(4), RobotMap.BACK_RIGHT_ANGLE_OFFSET);
}
