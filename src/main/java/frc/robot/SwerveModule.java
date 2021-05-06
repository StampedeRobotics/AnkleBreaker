package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveModule {
    public final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;
    private double steeringAngle;
    private int driveScalar = 1;
    private final CANCoder turningEncoder;
    private double stateDrive = 0;

    double driveOutput;

    private final PIDController turningPIDController;
    private final double angleOffset;

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorObject ID for the drive motor.
     * @param steerMotorObject ID for the turning motor.
     */

    public SwerveModule(WPI_TalonFX driveMotorObject, WPI_TalonFX steerMotorObject, CANCoder steerEncoderObject,
            double encoderOffset) {
        double kPSpecial;
        double kD;
        if (steerMotorObject.getDeviceID() == 4) {
            kPSpecial = .8;
            kD = 0.02;
            // kModuleMaxAngularVelocity = Math.PI;
            // kModuleMaxAngularAcceleration = Math.PI/2;
        } else {
            kPSpecial = 4 / Math.PI;
            kD = 0.02;
            // kD = 0.02;
        }

        turningPIDController = new PIDController(kPSpecial, 0, kD);

        driveMotor = driveMotorObject;
        turningMotor = steerMotorObject;
        turningEncoder = steerEncoderObject;

        angleOffset = encoderOffset;

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation
        // of the wheel divided by the encoder
        // loolution.

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * wpi::math::pi)
        // divided by the encoder resolution.
        // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public double getTalonFXRate() {
        double ticksPerSec = driveMotor.getSelectedSensorVelocity(0) * 10;
        double revsPerSec = ticksPerSec / (RobotMap.ENCODER_RESOLUTION * 8.307692307692308);
        double metersPerSec = revsPerSec * 2 * Math.PI * RobotMap.WHEEL_RADIUS;
        return metersPerSec;
    }

    public double getTalonFXPos() {
        double ticks = driveMotor.getSelectedSensorPosition(0);
        double revs = ticks / (RobotMap.ENCODER_RESOLUTION * 8.307692307692308);
        double meters = revs * 2 * Math.PI * RobotMap.WHEEL_RADIUS;
        return meters;
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getTalonFXRate(), new Rotation2d(getAngle()));
    }

    public void readAngle() {
        steeringAngle = turningEncoder.getPosition();
    }

    public double getAngle() {
        return steeringAngle;
    }

    public double angleSupp(double angle) {
        if (angle > Math.PI) {
            return angle - 2 * Math.PI;
        } else if (angle < -Math.PI) {
            return angle + 2 * Math.PI;
        } else {
            return angle;
        }
    }

    public double bound(double setpoint) {
        double dTheta = setpoint - getAngle();
        double trueDTheta = Math.IEEEremainder(dTheta, Math.PI);
        // double angleToReturn;

        /*
         * if (Math.abs(Math.IEEEremainder(getAngle() + trueDTheta, 2 * Math.PI) -
         * Math.IEEEremainder(setpoint, 2 * Math.PI)) < .01)
         */
        if (Math.abs(Math.IEEEremainder(trueDTheta + getAngle() - setpoint, 2 * Math.PI)) < .001) {
            driveScalar = 1;
        } else {
            driveScalar = -1;
        }

        // Ensure Angle is between -pi and pi
        if (Math.abs(trueDTheta) < Math.PI / 2) {
            return angleSupp(getAngle() + trueDTheta);
        } else {
            return angleSupp(getAngle() + (trueDTheta - Math.PI));
        }
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        readAngle();
        driveScalar = 1;
        double setpoint = bound(state.angle.getRadians());
        double currentAngle = getAngle();
        var turnOutput = turningPIDController.calculate(currentAngle, setpoint);
        stateDrive = state.speedMetersPerSecond;
        driveOutput = state.speedMetersPerSecond / Constants.MAX_SPEED * driveScalar;

        if (Math.abs(driveOutput) == 0) {
            turnOutput = 0;
        }

        turningMotor.set(turnOutput);
        driveMotor.set(driveOutput);
    }
}