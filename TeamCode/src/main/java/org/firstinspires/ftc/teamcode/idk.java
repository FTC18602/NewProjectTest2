package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
@TeleOp(name = "Robot_arm_test_bucket (Blocks to Java)")
public class idk extends LinearOpMode {
    FtcDashboard dashboard;


    private DcMotor arm;
    private DcMotor front_right_port_2;
    private DcMotor back_right_port_1;
    private CRServo sweeper;
    private DcMotor lift;
    private Servo launcher;
    private Servo bucket;
    private DcMotor front_left_port_0;

    public static double test = 7;
    private DcMotor back_left_port_3;
    public static double drivemode = 1;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */

    @Override
    public void runOpMode() {
        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        arm = hardwareMap.get(DcMotor.class, "arm");
        front_right_port_2 = hardwareMap.get(DcMotor.class, "front_right_port_2");
        back_right_port_1 = hardwareMap.get(DcMotor.class, "back_right_port_1");
        sweeper = hardwareMap.get(CRServo.class, "sweeper");
        lift = hardwareMap.get(DcMotor.class, "lift");
        launcher = hardwareMap.get(Servo.class, "launcher");
        bucket = hardwareMap.get(Servo.class, "bucket");
        front_left_port_0 = hardwareMap.get(DcMotor.class, "front_left_port_0");
        back_left_port_3 = hardwareMap.get(DcMotor.class, "back_left_port_3");

        // Put initialization blocks here.
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right_port_2.setDirection(DcMotor.Direction.REVERSE);
        back_right_port_1.setDirection(DcMotor.Direction.REVERSE);
        sweeper.setDirection(CRServo.Direction.REVERSE);
        // bucket should rotate opposite of arm when lifting
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad2.left_trigger != 0) {
                    ((DcMotorEx) arm).setVelocity(500);
                    arm.setTargetPosition((int) (arm.getCurrentPosition() - gamepad2.left_trigger * 50));
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                if (gamepad2.right_trigger != 0) {
                    ((DcMotorEx) arm).setVelocity(500);
                    arm.setTargetPosition((int) (arm.getCurrentPosition() + gamepad2.right_trigger * 50));
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                // Put loop blocks here.
                // This function is for bucket balancing and dumping
                // Buck should move in opposite rotation of arm whil lifting
                // Arm is on a motor with infinet "click" position, Bucket is on a servo that will move from 0-1.
                if (gamepad2.dpad_up) {
                    launcher.setPosition(0.35);
                } else {
                    launcher.setPosition(0.65);
                }
                if (gamepad2.a == false) {
                    if (arm.getCurrentPosition() < -10 && arm.getCurrentPosition() > -230 && true) {
                        bucket.setPosition(1);
                    } else {
                        bucket.setPosition(0.95 + arm.getCurrentPosition() / (300 * (435 / 60)));
                    }
                } else {
                    bucket.setPosition(1.3 + arm.getCurrentPosition() / (300 * (435 / 60)));
                }
                // Lift mechanism contorl blocks
                if (gamepad1.left_trigger != 0) {
                    lift.setPower(-gamepad1.left_trigger);
                } else {
                    lift.setPower(gamepad1.right_trigger);
                }
                holonomic(gamepad1.right_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
                // Put loop blocks here.
                if (gamepad2.left_bumper == true) {
                    sweeper.setPower(2);
                } else if (gamepad2.right_bumper == true) {
                    sweeper.setPower(-2);
                } else {
                    sweeper.setPower(0);
                }
                telemetry.addData("Button Y", gamepad1.y);
                telemetry.addData("Lift power", arm.getPower());
                telemetry.addData("Arm TargetPos", arm.getTargetPosition());
                telemetry.addData("bucket pos", bucket.getPosition());
                telemetry.addData("arm power", arm.getPower());
                telemetry.addData("Arm pos", arm.getCurrentPosition());
                telemetry.addData("arm CurrentPosition", arm.getCurrentPosition());
                telemetry.addData("Button A", gamepad1.a);
                telemetry.update();
                dashboardTelemetry.addData("x", 3.7);
                dashboardTelemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void holonomic(float vertica, float horizonta, float pivo) {
        double Vertical;
        double Horizontal;
        double Pivot;
        if (gamepad1.a){
            drivemode = 1;
        }
        if (gamepad1.b){
            drivemode = 0.7;
        }
        if (gamepad1.x){
            drivemode = 0.4;
        }
        Vertical = vertica * drivemode;
        Horizontal = -(horizonta * drivemode);
        Pivot = -(pivo * drivemode);
        front_right_port_2.setPower(-Pivot + (Vertical - Horizontal));
        back_right_port_1.setPower(-Pivot + Vertical + Horizontal);
        front_left_port_0.setPower(Pivot + Vertical + Horizontal);
        back_left_port_3.setPower(Pivot + (Vertical - Horizontal));
        // Put loop blocks here.
    }
}