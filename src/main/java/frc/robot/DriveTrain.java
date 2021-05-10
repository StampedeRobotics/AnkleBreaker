package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class DriveTrain {
    private static final Translation2d frontLeftTranslation = new Translation2d(0.155, 0.155);
    private static final Translation2d frontRightTranslation = new Translation2d(0.155, -0.155);
    private static final Translation2d backLeftTranslation = new Translation2d(-0.155, 0.155);
    private static final Translation2d backRightTranslation = new Translation2d(-0.155, 0.155);

    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(frontLeftTranslation,
            frontRightTranslation, backLeftTranslation, backRightTranslation);

    public static final SwerveDriveOdometry ODOMETRY = new SwerveDriveOdometry(KINEMATICS,
            new Rotation2d(RobotMap.GYRO.getYaw()));

    public static void drive() {
        // Get speeds from controller
        double xSpeed = RobotMap.CONTROLLER.getX(Hand.kLeft) * Constants.MAX_SPEED;
        double ySpeed = RobotMap.CONTROLLER.getY(Hand.kLeft) * Constants.MAX_SPEED;
        // Get the rotation from the controller
        double rotation = RobotMap.CONTROLLER.getX(Hand.kRight) * Constants.MAX_ANGULAR_SPEED;

        SwerveModuleState[] moduleStates = KINEMATICS.toSwerveModuleStates(
                Constants.FIELD_RELATIVE ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, getAngle())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotation));

        SwerveDriveKinematics.normalizeWheelSpeeds(moduleStates, Constants.MAX_SPEED);

        RobotMap.FRONT_LEFT.setState(moduleStates[0]);
        RobotMap.FRONT_RIGHT.setState(moduleStates[1]);
        RobotMap.BACK_LEFT.setState(moduleStates[2]);
        RobotMap.BACK_RIGHT.setState(moduleStates[3]);
    }

    private static Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-RobotMap.GYRO.getYaw());
    }
}
