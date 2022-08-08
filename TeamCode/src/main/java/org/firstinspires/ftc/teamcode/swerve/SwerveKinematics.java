package org.firstinspires.ftc.teamcode.swerve;

public class SwerveKinematics {

    private double trackWidth;

    private double[] moduleAngleSpeed = new double[2];
    private double[] wheelDriveSpeed = new double [2];

    //public double[] wheelVelocities = new double[2];
    //public double[] wheelAngles = new double[2];

    public void calculateKinematics(double rotationPower, double strafePower, double forwardPower) {
        if (rotationPower != 0 || strafePower != 0 || forwardPower != 0) {

            double v1X = strafePower;
            double v1Y = forwardPower - rotationPower * trackWidth / 2.0;

            double v2X = strafePower;
            double v2Y = forwardPower + rotationPower * trackWidth / 2.0;

            /* double vectorMath1 = strafePower - (rotationPower * wheelBase / 2);
            double vectorMath2 = strafePower + (rotationPower * wheelBase / 2);
            double vectorMath3 = forwardPower - (rotationPower * trackWidth / 2); */

            moduleAngleSpeed[0] = Math.atan2(v1Y, v1X); // module 1 angle
            wheelDriveSpeed[0] = Math.hypot(v1Y, v1X); // module 1 speed

            moduleAngleSpeed[1] = Math.atan2(v2Y, v2X); // module 2 angle
            wheelDriveSpeed[2] = Math.hypot(v2Y, v2X); // module 2 speed

            scalePowers();
        }
    }

    public double[] getWheelAngles() {
        double[] wheelAngles = new double[2];
        wheelAngles[0] = moduleAngleSpeed[0];
        wheelAngles[1] = moduleAngleSpeed[1];
        return wheelAngles;
    }

    public double[] getWheelVelocities() {
        double[] wheelVelocities = new double[2];
        wheelVelocities[0] = wheelDriveSpeed[1];
        wheelVelocities[1] = wheelDriveSpeed[3];
        return wheelVelocities;
    }

    public void scalePowers() {
        double maxPower = 0;
        for (int i = 0; i < 2; i++) {
            if (Math.abs(wheelDriveSpeed[0]) > maxPower)
                maxPower = Math.abs(wheelDriveSpeed[0]);
            if (Math.abs(wheelDriveSpeed[1]) > maxPower)
                maxPower = Math.abs(wheelDriveSpeed[1]);
        }
        if (maxPower > 1.0) {
            wheelDriveSpeed[0] /= maxPower;
            wheelDriveSpeed[1] /= maxPower;
        }
    }
    //Vector2d(vx - omega * y, vy + omega * x),
    //Vector2d(vx - omega * y, vy - omega * x),

    public void setTrackWidth(double trackWidth) { this.trackWidth = trackWidth; }
    //public void setWheelBase(double wheelBase) { this.wheelBase = wheelBase; }
}
