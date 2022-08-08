package org.firstinspires.ftc.teamcode.swerve;

public class SwerveConstants {
    public static final double radius = 13.2283464567; //336mm trackwidth
    public static final double trackWidth = 13.2283464567; //radius * Math.cos(Math.toRadians(45)); //9.35385501181
    public static final double wheelBase = radius * Math.sin(Math.toRadians(45)); //9.35385195502
}
