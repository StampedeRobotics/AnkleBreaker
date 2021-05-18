package frc.robot;

public class Constants {
  public static final boolean FIELD_RELATIVE = true;

  public static final double MAX_SPEED = 4.3;
  public static final double MAX_ACCELERATION = MAX_SPEED;
  public static final double MAX_ANGULAR_SPEED = 4 * Math.PI;
  public static final double MAX_ANGULAR_ACCELERATION = 4 * Math.PI;

  public static final int CANCODER_RESOLUTION = 4096;
  public static final int TALONFX_RESOLUTION = 2048;
  public static final double WHEEL_RADIUS = 0.051;

  public static final double BACK_RIGHT_ANGLE_OFFSET = 2.01 + Math.PI;
  public static final double BACK_LEFT_ANGLE_OFFSET = -0.46;
  public static final double FRONT_RIGHT_ANGLE_OFFSET = -2.56 + Math.PI;
  public static final double FRONT_LEFT_ANGLE_OFFSET = 0.07;

}
