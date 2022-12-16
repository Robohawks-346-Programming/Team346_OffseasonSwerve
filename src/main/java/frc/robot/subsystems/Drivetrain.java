package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase{
    
    SwerveDriveOdometry odometry; 

    SwerveModule frontLeft = new SwerveModule(
            Constants.FRONT_LEFT_DRIVE_ID,
            Constants.FRONT_LEFT_TURN_ID,
            Constants.FRONT_LEFT_ENCODER_ID,
            Constants.FRONT_LEFT_TURN_OFFSET
        );

    SwerveModule frontRight = new SwerveModule(
            Constants.FRONT_RIGHT_DRIVE_ID,
            Constants.FRONT_RIGHT_TURN_ID,
            Constants.FRONT_RIGHT_ENCODER_ID,
            Constants.FRONT_RIGHT_TURN_OFFSET
        ); 

    SwerveModule backLeft = new SwerveModule(
            Constants.BACK_LEFT_DRIVE_ID,
            Constants.BACK_LEFT_TURN_ID,
            Constants.BACK_LEFT_ENCODER_ID,
            Constants.BACK_LEFT_TURN_OFFSET
        );

    SwerveModule backRight = new SwerveModule(
            Constants.BACK_RIGHT_DRIVE_ID,
            Constants.BACK_RIGHT_TURN_ID,
            Constants.BACK_RIGHT_ENCODER_ID,
            Constants.BACK_RIGHT_TURN_OFFSET
        );

    SwerveModule[] modules = {frontLeft, frontRight, backLeft, backRight};

    ADIS16448_IMU gyro = new ADIS16448_IMU();

    public Drivetrain() {
        gyro.reset(); //Ask if this would work
        
        odometry = new SwerveDriveOdometry(Constants.DRIVE_KINEMATICS, getHeading());

        for( SwerveModule module : modules) {
            module.resetDistance();
        }
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose2d) {
        odometry.resetPosition(pose2d, getHeading());
    }

    public void drive(ChassisSpeeds speed, ChassisSpeeds percent) {
        if(speed.vxMetersPerSecond == 0 && speed.vyMetersPerSecond == 0 && speed.omegaRadiansPerSecond == 0) {
            brake();
            return;
        }

        SwerveModuleState[] swerveStates = Constants.DRIVE_KINEMATICS.toSwerveModuleStates(speed);

        normalDrive(swerveStates, Constants.MAX_VELOCITY, percent);
        setModuleState(swerveStates);
    }

    public void brake() {
        for (SwerveModule module : modules) {
            module.setState(new SwerveModuleState(0, module.getState().angle));
        }
    }

    public void normalDrive(SwerveModuleState[] states, double maxVelocity, ChassisSpeeds percent) {
        double x = percent.vxMetersPerSecond;
        double y = percent.vyMetersPerSecond;
        double theta = percent.omegaRadiansPerSecond;
        double maxSpeed = 0.0;
        for(SwerveModuleState moduleState : states) {
            if (Math.abs(moduleState.speedMetersPerSecond) > maxSpeed) {
                maxSpeed = Math.abs(moduleState.speedMetersPerSecond);
            }
        }
        double a = Math.max(Math.hypot(x, y), Math.abs(theta));
        if (maxSpeed != 0.0) {
            for (SwerveModuleState moduleState : states) {
                moduleState.speedMetersPerSecond *= (a* maxVelocity / maxSpeed);
            }
        }
    }

    public void setModuleState(SwerveModuleState[] states) {
        for (int i = 0; i<=3; i++) {
            modules[i].setState(states[i]);
        }
    }

    public SwerveModuleState[] getModuleState() {
        SwerveModuleState[] states = new SwerveModuleState[4];

        for (int i=0; i<=3; i++) {
            states[i++] = modules[i].getState(); 
        }

        return states;
    }

    public void resetEncoders() {
        for (SwerveModule module : modules) {
            module.resetDistance();
        }
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public Rotation2d getHeading() {
        double rawYaw = gyro.getAngle();
        double calcYaw = rawYaw;
        if(0.0 > rawYaw) {
            calcYaw +=360.0;
        }
        return Rotation2d.fromDegrees(-calcYaw);
    }

    public void syncEncoders() {
        for(SwerveModule module : modules) {
            module.syncTurnEncoders();
        }
    }
}

