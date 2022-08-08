package org.firstinspires.ftc.teamcode.swerve;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperfunctions.AS5600;
import org.firstinspires.ftc.teamcode.helperfunctions.MathFunctions;
import org.firstinspires.ftc.teamcode.helperfunctions.PID.SwerveRotationPID;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

public class SwerveDrive {
    public SwerveKinematics swerveKinematics;

    private HardwareMap hardwareMap;

    private DcMotorEx right_motor1, right_motor2, left_motor1, left_motor2;

    // pid constants
    ElapsedTime right_time = new ElapsedTime();
    ElapsedTime left_time = new ElapsedTime();

    private AS5600 right_as5600, left_as5600;
    public SwerveRotationPID right_module, left_module;

    private SwerveModule rightModule, leftModule;
    public SwerveModule[] swerveModules = new SwerveModule[2];

    VoltageSensor voltageSensor;

    private enum ZEROBEHAVIOR {
        RESPONSIVE,
        LOCK
    }

    ZEROBEHAVIOR zerobehavior;

    public SwerveDrive(HardwareMap hardwareMap) {
        swerveKinematics = new SwerveKinematics();
        swerveKinematics.setTrackWidth(SwerveConstants.trackWidth);

        this.hardwareMap = hardwareMap;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        right_motor1 = hardwareMap.get(DcMotorEx.class, "right1");
        right_motor2 = hardwareMap.get(DcMotorEx.class, "right2");
        left_motor1 = hardwareMap.get(DcMotorEx.class, "left1");
        left_motor1 = hardwareMap.get(DcMotorEx.class, "left2");

        right_as5600 = new AS5600(hardwareMap, "Right_analog", 2.621, 3.291, 0.001);
        left_as5600 = new AS5600(hardwareMap, "Left_analog", 3.251, 3.282, 0.001);

        right_module = new SwerveRotationPID(0.0, 0.0, 0.0, 0.0, right_time);
        left_module = new SwerveRotationPID(0.0, 0.0, 0.0, 0.0, left_time);

        rightModule = new SwerveModule(right_motor1, right_motor2, right_module, right_as5600);
        leftModule = new SwerveModule(left_motor1, left_motor2, left_module, left_as5600);

        swerveModules[0] = rightModule;
        swerveModules[1] = leftModule;

        zerobehavior = ZEROBEHAVIOR.RESPONSIVE;
    }

    public void setMotorPowers(double rotation, double strafe, double forward) {
        swerveKinematics.calculateKinematics(rotation, strafe, forward);
        double[] rotationPowers = swerveKinematics.getWheelAngles();
        double[] drivePowers = swerveKinematics.getWheelVelocities();

        double deadzone = 0.1;

        if (Math.abs(strafe) < deadzone && Math.abs(forward) < deadzone && Math.abs(rotation) < deadzone) {
            drivePowers[0] = 0;
            drivePowers[1] = 0;
        }

        double rightTargetRotPower = rightModule.rotationPower(rightModule, rotationPowers[0]);
        double rightTargetDrivePower = drivePowers[0];

        rightModule.setPower(rightTargetDrivePower, rightTargetRotPower);

        double leftTargetRotPower = rightModule.rotationPower(rightModule, rotationPowers[1]);
        double leftTargetDrivePower = drivePowers[1];

        leftModule.setPower(leftTargetDrivePower, leftTargetRotPower);
    }

}