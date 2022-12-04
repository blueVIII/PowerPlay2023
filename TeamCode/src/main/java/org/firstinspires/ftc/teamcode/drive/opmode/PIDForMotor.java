package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.qualcomm.ftccommon.FtcWifiDirectChannelSelectorActivity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.FtcRobotControllerServiceState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class PIDForMotor extends LinearOpMode {

    DcMotorEx liftMotor;
    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    public static double Kp = 0.03;
    public static double Ki = 0.0;
    public static double Kd = 0.0002;
    public static double Kg = 0.05;

    public static int targetPosition;
    //public static double targetPosition = 1000;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        //TelemetryPacket packet = new TelemetryPacket();

        //dashboard.setTelemetryTransmissionInterval(25);
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        targetPosition = 1000;

        while(opModeIsActive()) {
            double power = returnPower(targetPosition, liftMotor.getCurrentPosition());
            //packet.put("power", power);
            //packet.put("position", liftMotor.getCurrentPosition());
            //packet.put("error", lastError);
            liftMotor.setPower(power);

            //dashboard.sendTelemetryPacket(packet);

            //drive.goToPositionLiftMotor(timer, Kp, Ki, Kd, targetPosition);
            //high goal = 2800 ticks
            //top cone = 400
        }
    }

    public double returnPower(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + Kg;

        return output;
    }
}
