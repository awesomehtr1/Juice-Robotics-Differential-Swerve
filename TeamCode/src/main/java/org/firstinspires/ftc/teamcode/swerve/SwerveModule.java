package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.MathFunctions;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

public class SwerveModule {
    public SwerveKinematics swerveKinematics;
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    public SwerveRotationPID pid;
    private AS5600 as5600;
    private boolean reverseDrive = false;

    private double targetAngle;
    private double speed;

    private double prevDrivePos;

    // swerve module wrapper for storing drive motor, rotation motor, rotation pid, and analog encoder
    // includes getter and setters
    // handles some low level swerve control
    public SwerveModule(DcMotorEx motor1, DcMotorEx motor2, SwerveRotationPID pid, AS5600 as5600) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.pid = pid;
        this.as5600 = as5600;
        prevDrivePos = 0;
    }

    public void setAngle(double angle) {
        targetAngle = angle;
        pid.setState(angle);
    }

    public void setPower(double drivePower, double rotationPower) {
        motor1.setPower(drivePower + rotationPower);
        motor2.setPower(drivePower - rotationPower);
    }

    // make pod rotate no more than 90 degrees
    public double angleOptimization(SwerveModule module, double targetAngle) {
        if(Math.abs(targetAngle - module.getAngle()) > Math.PI/2) {
            targetAngle += Math.PI;
            MathFunctions.angleWrap(targetAngle);
            module.setReverseDrive(true);
        }
        else
            module.setReverseDrive(false);
        return targetAngle;
    }

    public double rotationPower(SwerveModule module, double targetAngle) {
        targetAngle = angleOptimization(module, targetAngle);
        module.setAngle(targetAngle);
        double rotPower = module.updatePID(module.getAngle());
        return rotPower;
    }

    public void setReverseDrive(boolean bool) { reverseDrive = bool; }

    // returns analog encoder angle; returns -pi to pi radian format
    public double getAngle() { return as5600.getAngle(); }

    // updates PID with current angle; returns power
    public double updatePID(double pos) { return pid.updatePID(pos); }

}