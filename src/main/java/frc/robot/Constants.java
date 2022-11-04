package frc.robot;
public final class Constants {

    public static final class DriveConstants {
        public static final int kLeftMotor1Port = 7;
        public static final int kLeftMotor2Port = 9;
        public static final int kRightMotor1Port = 6;
        public static final int kRightMotor2Port = 8;

        public static final int kLeftEncoderChannelA = 0;
        public static final int kLeftEncoderChannelB = 1;
        public static final int kRightEncoderChannelA = 2;
        public static final int kRightEncoderChannelB = 3;
        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.128 * Math.PI;
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;

        public static final double kAutoDriveForwardSpeed = 0.5;
        public static final double kAutoDriveForwardDistance = 1.5;
    }

    public static final class ShooterConstants {
        public static final int kLeftMotorPort = 4;
        public static final int kRightMotorPort = 5;
        public static final int kConveyorMotorPort = 3;
        public static final int kIntakeMotorPort = 0;
        public static final int kAimMotorPort = 1;
        public static final int kTriggerMotorPort = 1;
        public static final int kIntakeUp = 1;
        public static final int kIntakeDown = 0;
        public static final int kMaxSpeed = 1;
        public static final double kMaxSpeedIntake = 0.2;
        public static final int kEncoderCPR = 8192;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse = 0.48/2048.;
        }  // namespace ShooterConstants

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double limelightMountAngleDegrees = 30.0;
        public static final double limelightLensHeight = 0.50;
        public static final double goalHeight = 2.64;
        public static final int kShooterAimButton = 1;
        public static final int kTriggerButton = 2;
        public static final int kAimForwardButton = 3;
        public static final int kAimRevertButton = 4;
        public static final int kConveyorButton = 5;
        public static final int kConveyorRevetButton = 6;
        public static final int kClimberRevertButton = 8;
        public static final int kIntakeButton = 9;
        public static final int kClimberButton = 12;
    }

}
