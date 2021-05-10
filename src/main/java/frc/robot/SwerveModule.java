package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

public class SwerveModule {
    private final int id;

    private final WPI_TalonFX driveMotor;
    private final TalonFXSensorCollection driveEncoder;
    private final PIDController drivePIDController = new PIDController(0.5, 0, 0.98);

    private final WPI_TalonFX turningMotor;
    private CANCoder turningEncoder;
    private final double turningEncoderOffset;
    private final ProfiledPIDController turningPIDController;

    private static final Constraints CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.MAX_ANGULAR_SPEED,
            Constants.MAX_ANGULAR_ACCELERATION);

    public SwerveModule(int id, WPI_TalonFX driveMotor, WPI_TalonFX turningMotor, CANCoder turningEncoder,
            double turningEncoderOffset) {
        this.id = id;
        this.driveMotor = driveMotor;
        this.driveEncoder = driveMotor.getSensorCollection();
        this.turningMotor = turningMotor;
        this.turningEncoder = turningEncoder;
        this.turningEncoderOffset = turningEncoderOffset;
        this.turningPIDController = new ProfiledPIDController(0.8, 0, 0, CONSTRAINTS);

        turningEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // driveEncoder.setDistancePerPulse(2 * Math.PI * Constants.WHEEL_RADIUS /
        // Constants.CANCODER_RESOLUTION);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * wpi::math::pi)
        // divided by the encoder resolution.
        // turningEncoder.setDistancePerPulse(2 * Math.PI /
        // Constants.TALONFX_RESOLUTION);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private double getTurnAngle() {
        return Math.toRadians(turningEncoder.getAbsolutePosition());
    }

    public void setState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getTurnAngle()));

        SmartDashboard.putNumber("Drive" + id, getTurnAngle());

        // Calculate the drive output from the drive PID controller.
        // double turnOutput =
        // drivePIDController.calculate(turningEncoder.getAbsolutePosition(),
        // state.angle.getDegrees());
        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = turningPIDController.calculate(getTurnAngle(), state.angle.getRadians());
        SmartDashboard.putNumber("Turn" + id, turnOutput);

        // SmartDashboard.putNumber("Drive" + id, turnOutput);
        // driveMotor.set(state.speedMetersPerSecond / Constants.MAX_SPEED);
        turningMotor.set(turnOutput);
    }
}