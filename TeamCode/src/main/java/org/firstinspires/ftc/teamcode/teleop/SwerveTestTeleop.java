package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.swerve.SwerveDrive;

@TeleOp(name = "Drive Only", group = "Teleop")
public class SwerveTestTeleop extends LinearOpMode {
    public void runOpMode(){
        PhotonCore.enable();
        SwerveDrive swerveDrive = new SwerveDrive(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            double rotation = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            double forward = -gamepad1.left_stick_y;
            swerveDrive.setMotorPowers(rotation, strafe, forward);

            telemetry.update();
        }
    }
}