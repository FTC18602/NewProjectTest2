package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@Autonomous(name = "test_auto")
public class test_auto extends LinearOpMode {

    private DcMotor back_left_port_3;
    private DcMotor back_right_port_1;
    private DcMotor front_right_port_2;
    private DcMotor front_left_port_0;
    private DcMotor dead_right_port_1;
    private DcMotor dead_back_port_2;
    double x =-1;
    double y;
    int position;
    Recognition myTfodRecognition;
    List<Recognition> myTfodRecognitions;
    VisionPortal myVisionPortal;
    TfodProcessor myTfodProcessor;
    boolean USE_WEBCAM;
    double drive_mode=0.5;
    RunToPosition gotopos;
    RunToPosition gotopos2;
    holonomic drive;
    int loopcounter = 0;
    boolean flag = false;

    @Override
    public void runOpMode() {
        // This 2023-2024 OpMode illustrates the basics of TensorFlow Object Detection, using
        // a custom TFLite object detection model.
        USE_WEBCAM = true;
        // Initialize TFOD before waitForStart.
        initTfod();
        Servo plane;
        back_left_port_3 = hardwareMap.get(DcMotor.class, "back_left_port_3");
        back_right_port_1 = hardwareMap.get(DcMotor.class, "back_right_port_1");
        front_right_port_2 = hardwareMap.get(DcMotor.class, "front_right_port_2");
        front_left_port_0 = hardwareMap.get(DcMotor.class, "front_left_port_0");
        dead_right_port_1 = hardwareMap.get(DcMotor.class, "dead_right_port_1");
        dead_back_port_2 = hardwareMap.get(DcMotor.class, "dead_back_port_2");
        gotopos = new RunToPosition();
        gotopos2 = new RunToPosition();
        drive = new holonomic();
        motormodes();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {

                if (gamepad1.dpad_down) {
                    // Temporarily stop the streaming session.
                    myVisionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    // Resume the streaming session if previously stopped.
                    myVisionPortal.resumeStreaming();
                }


                // Iterate through list and call a function to display info for each recognized object.
                if (x==-1) {
                    while(!gotopos.atpos && opModeIsActive()) {
                        gotopos.Run(dead_right_port_1.getCurrentPosition(), dead_back_port_2.getCurrentPosition(), 0, -8, 1, 1);
                        drive.run(gotopos.Xpower, gotopos.Ypower, 0, 0.2);
                        ((DcMotorEx) front_right_port_2).setVelocity(drive.FrontRight() * 2700);
                        ((DcMotorEx) front_left_port_0).setVelocity(drive.FrontLeft() * 2700);
                        ((DcMotorEx) back_right_port_1).setVelocity(drive.BackRight() * 2700);
                        ((DcMotorEx) back_left_port_3).setVelocity(drive.BackLeft() * 2700);
                    }
                    while (loopcounter < 150000 && opModeIsActive()) {
                        myTfodRecognitions = myTfodProcessor.getRecognitions();
                        for (Recognition myTfodRecognition_item2 : myTfodRecognitions) {
                            myTfodRecognition = myTfodRecognition_item2;
                            x = (myTfodRecognition.getLeft() + myTfodRecognition.getRight()) / 2;
                            y = (myTfodRecognition.getTop() + myTfodRecognition.getBottom()) / 2;
                        }
                        telemetry.addData("X", x);
                        telemetry.addData("Y", y);
                        telemetry.addData("loopcounter", loopcounter++);
                        telemetry.update();
                    }


                    if (x <= 300 && !(x < 1))
                        position = 1;
                    else if (x > 300)
                        position = 0;
                    else if (x == -1) {
                        position = 2;
                        x = 0;
                    }
                    gotopos.atpos = false;
                }
// testing my branches

                while(!gotopos.atpos && opModeIsActive()) {
                    if (position == 0)
                        gotopos.Run(dead_right_port_1.getCurrentPosition(), dead_back_port_2.getCurrentPosition(), -9, -28.5, 1, 1);
                    else if (position == 1)
                        gotopos.Run(dead_right_port_1.getCurrentPosition(), dead_back_port_2.getCurrentPosition(), 0, -30, 1, 2);
                    else if (position == 2)
                        gotopos.Run(dead_right_port_1.getCurrentPosition(), dead_back_port_2.getCurrentPosition(), 16.5, -26, 1, 2);
                    //else if (position == 2)
                    //gotopos.Run(dead_right_port_1.getCurrentPosition(), dead_back_port_2.getCurrentPosition(), -12, -12, 1, 1);


                    drive.run(gotopos.Xpower, gotopos.Ypower, 0, 0.2);
                    ((DcMotorEx) front_right_port_2).setVelocity(drive.FrontRight() * 2700);
                    ((DcMotorEx) front_left_port_0).setVelocity(drive.FrontLeft() * 2700);
                    ((DcMotorEx) back_right_port_1).setVelocity(drive.BackRight() * 2700);
                    ((DcMotorEx) back_left_port_3).setVelocity(drive.BackLeft() * 2700);
                }//branch test
                if (gotopos.atpos) {
                    gotopos2.Run(dead_right_port_1.getCurrentPosition(), dead_back_port_2.getCurrentPosition(), 0, 4, 1, 2);
                    if (!gotopos2.atpos) {
                        drive.run(gotopos2.Xpower, gotopos2.Ypower, 0, 0.2);
                        ((DcMotorEx) front_right_port_2).setVelocity(drive.FrontRight() * 2700);
                        ((DcMotorEx) front_left_port_0).setVelocity(drive.FrontLeft() * 2700);
                        ((DcMotorEx) back_right_port_1).setVelocity(drive.BackRight() * 2700);
                        ((DcMotorEx) back_left_port_3).setVelocity(drive.BackLeft() * 2700);
                    }
                }
                telemetry.addData("posx", gotopos.PosX);
                telemetry.addData("posy", gotopos.PosY);
                telemetry.addData("deltax", gotopos.DeltaX);
                telemetry.addData("posy", gotopos.DeltaY);
                telemetry.addData("Xpower", gotopos.Xpower);
                telemetry.addData("Ypower", gotopos.Ypower);
                telemetry.addData("X", x);
                telemetry.addData("Y", y);
                telemetry.update();
            }
        }
    }

    private void motormodes() {
        back_left_port_3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_port_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left_port_0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right_port_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dead_back_port_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dead_right_port_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_port_3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right_port_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left_port_0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right_port_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right_port_2.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right_port_1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void initTfod() {
        TfodProcessor.Builder myTfodProcessorBuilder;
        VisionPortal.Builder myVisionPortalBuilder;

        // First, create a TfodProcessor.Builder.
        myTfodProcessorBuilder = new TfodProcessor.Builder();
        // Set the name of the file where the model can be found.
        myTfodProcessorBuilder.setModelFileName("model_cameravisionV2.tflite");
        // Set the full ordered list of labels the model is trained to recognize.
        myTfodProcessorBuilder.setModelLabels(JavaUtil.createListWith("BLUE Elm", "RED Elm"));
        // Set the aspect ratio for the images used when the model was created.
        myTfodProcessorBuilder.setModelAspectRatio(16 / 9);
        // Create a TfodProcessor by calling build.
        myTfodProcessor = myTfodProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        myVisionPortalBuilder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            // Use a webcam.
            myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            // Use the device's back camera.
            myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        }
        // Add myTfodProcessor to the VisionPortal.Builder.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);
        // Create a VisionPortal by calling build.
        myVisionPortal = myVisionPortalBuilder.build();
    }
    private void telemetryTfod() {


        // Get a list of recognitions from TFOD.
        myTfodRecognitions = myTfodProcessor.getRecognitions();
        telemetry.addData("# Objects Detected", JavaUtil.listLength(myTfodRecognitions));
        // Iterate through list and call a function to display info for each recognized object.
        for (Recognition myTfodRecognition_item2 : myTfodRecognitions) {
            myTfodRecognition = myTfodRecognition_item2;
            // Display info about the recognition.
            telemetry.addLine("");
            // Display label and confidence.
            // Display the label and confidence for the recognition.
            telemetry.addData("Image", myTfodRecognition.getLabel() + " (" + JavaUtil.formatNumber(myTfodRecognition.getConfidence() * 100, 0) + " % Conf.)");
            // Display position.

            // Display the position of the center of the detection boundary for the recognition
            telemetry.addData("- Position", JavaUtil.formatNumber(x, 0) + ", " + JavaUtil.formatNumber(y, 0));
            // Display size
            // Display the size of detection boundary for the recognition
            telemetry.addData("- Size", JavaUtil.formatNumber(myTfodRecognition.getWidth(), 0) + " x " + JavaUtil.formatNumber(myTfodRecognition.getHeight(), 0));
        }
    }

}