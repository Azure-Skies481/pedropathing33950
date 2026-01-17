package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ShootingHelp {
    private final double maxSpeed = 2800;
    private final double feedback = 0.008;

    public double getCurrentRPM(DcMotorEx motor){
        return (motor.getVelocity() * 60) / 28;
    }

    public double getPID(DcMotorEx motor, double power){
        double actualSpeed= motor.getVelocity();
        return feedback * (power-actualSpeed) + actualSpeed/maxSpeed;
    }



}
