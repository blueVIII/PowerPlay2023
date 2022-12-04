package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class PIDController extends LinearOpMode {
    double integralSum = 0;
    double Kp = 0.05;
    double Ki = 0;
    double Kd = 0;
    double Kg = 0.2;

    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private SampleMecanumDrive drive;
    public PIDController (SampleMecanumDrive drive) {
        drive = this.drive;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
    }

    public void PIDControl(double reference) {
        drive.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double encoderPosition = drive.liftMotor.getCurrentPosition();
        while (encoderPosition < 2000) {
            // obtain the encoder position
            encoderPosition = drive.liftMotor.getCurrentPosition();
            // calculate the error
            double error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + Kg;

            drive.liftMotor.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();

        }
    }
}
