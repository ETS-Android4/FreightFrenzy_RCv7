package org.firstinspires.ftc.teamcode.Main;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.commands.core.LynxReadVersionStringResponse;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


//Road Runner Imports - Lucian

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.Autonomie.DetectObject;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.nio.file.attribute.FileOwnerAttributeView;
import java.util.function.Function;

@TeleOp(name = "Main_Driving", group = "main")

public class Driving1 extends LinearOpMode {

    //Motoare Brat, Slidere, Carusel, Colector
    private DcMotorEx motor_brat;
    private DcMotorEx motor_slider;
    private DcMotorEx motor_carusel;
    private DcMotorEx motor_colector;

    //Valoare Pentru Inversare Orientare Driving

    private SampleMecanumDrive mecanum_drive;

    //Variabila Dashboard
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private CRServo holder;

    //Variabila cu clasa de functii
    private functions fx = new functions();

    OpenCvWebcam webcam;
    DetectObject pipe_line = new DetectObject();
    @Override
    public void runOpMode() throws InterruptedException
    {
        //Webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);

        webcam.setPipeline(pipe_line);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });



        //Init motor pentru inclinare brat
        motor_brat = hardwareMap.get(DcMotorEx.class, "brat");

        motor_brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_brat.setDirection(DcMotorSimple.Direction.REVERSE);

        //Init pentru motor miscare de extindere si retragere
        motor_slider = hardwareMap.get(DcMotorEx.class, "slider");

        motor_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_slider.setDirection(DcMotorSimple.Direction.FORWARD);

        //Init pentru motorul are se cupa de aruselul cu rate
        motor_carusel = hardwareMap.get(DcMotorEx.class, "carusel");

        motor_carusel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_carusel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_carusel.setDirection(DcMotorSimple.Direction.FORWARD);

        //Init pentru motorul care se ocupa cu colectarea elementelor
        motor_colector = hardwareMap.get(DcMotorEx.class, "colector");

        motor_colector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_colector.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_colector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_colector.setDirection(DcMotorSimple.Direction.REVERSE);


        //Init pentru servo-ul de la holder de elemente
        holder = hardwareMap.crservo.get("holder");
        holder.setDirection(CRServo.Direction.REVERSE);


        //Init pentru mecanum drive RR
        mecanum_drive = new SampleMecanumDrive(hardwareMap);

        mecanum_drive.setPoseEstimate(new Pose2d(0, 0, 0));

        //Before start
        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Apasa pe buton!", "");
            telemetry.update();
        }


        boolean last_it_orientation = false;
        boolean curr_it_orientation = false;
        boolean tgl_orientation = false;
        int orientation_drive = -1;
        //Intrare in program
        while (opModeIsActive())
        {
            //Schimbare orientare robot
            last_it_orientation = curr_it_orientation;
            curr_it_orientation = gamepad2.x;

            if (curr_it_orientation && !last_it_orientation)
                orientation_drive *= -1;

            //Setare driving pe controller
            mecanum_drive.setWeightedDrivePower(
                    new Pose2d(
                            orientation_drive*gamepad2.left_stick_y,
                            orientation_drive*gamepad2.left_stick_x,
                            -gamepad2.right_stick_x
                    )
            );
            mecanum_drive.update();

            //Functiile de baza
            fx.brat(true);
            fx.slider(true);
            fx.carusel();
            fx.colector();
            fx.holder_cr(false);
            fx.reset_brat();

            telemetry.update();
        }

    }


    //Clasa functii de baza robot
    class functions {
        //deprecated
        private int button_sleep = 135;

        //Valoare care retine daca bratul este in reset sau nu
        private boolean resetting_brat = false;

        //Limita de jos si de sus a bratului ca inclinare
        private final int upper_limit_brat = 1400;
        private final int lower_limit_brat = -3500;

        //Valoare care retine daca bratul este destul de extins pentru inclinare si distanta de clear
        private boolean clear_brat = false;
        private int dist_min_ext = 900;

        private double put_brat = 0.8;

        //Functia de inclinare
        public void brat(boolean logs) {
            if (poz_slider >= dist_min_ext)
                clear_brat = true;
            else
                clear_brat = false;

            if (poz_slider >= 4000)
                put_brat = 0.7;
            else
                put_brat = 1;

            if (resetting_brat == false) {
                if (gamepad1.dpad_down &&  motor_brat.getCurrentPosition()*-1 >= lower_limit_brat && clear_brat)
                    motor_brat.setPower(put_brat);
                else if (gamepad1.dpad_up && motor_brat.getCurrentPosition()*-1 <= upper_limit_brat  && clear_brat)
                    motor_brat.setPower(-put_brat);
                else
                    motor_brat.setPower(0);
            }

            if (logs) {
                telemetry.addData("Pozitie Curenta Motor Brat: ", motor_brat.getCurrentPosition()*-1);
                telemetry.addData("Putere Teoretica Motor Brat: ", put_brat);
                telemetry.addData("Putere Practica Motor Brat: ", motor_brat.getPower());
            }
        }

        //Functia de resetare brat si slider
        boolean last_it_reset = false;
        boolean curr_it_reset = false;
        public void reset_brat() {
            last_it_reset = curr_it_reset;
            curr_it_reset = gamepad1.b;

            if (curr_it_reset && !last_it_reset) {
                resetting_brat = true;
                motor_brat.setTargetPosition(0);
                motor_slider.setTargetPosition(0);

                motor_brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (motor_brat.isBusy())
                    motor_brat.setPower(1);

                if (motor_slider.isBusy())
                    motor_slider.setPower(1);
            }

            if (!motor_slider.isBusy() && !motor_brat.isBusy())
                resetting_brat = false;

            if (resetting_brat == false) {
                motor_brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }

        //Pozitie si limite slidere de extindere si retragere
        private int poz_slider = 0;
        private int upper_limit_slider = 5350;
        private int lower_limit_slider = 20;
        private double put_slider = 1;

        //Functia pentru motoarele de la slider extindere/retragere
        public void slider(boolean logs) {
            poz_slider = motor_slider.getCurrentPosition()*-1;

            if (resetting_brat == false) {
                if (gamepad1.right_trigger >= 0.3 && poz_slider <= upper_limit_slider) {
                    motor_slider.setPower(-put_slider);
                } else if (gamepad1.left_trigger >= 0.3 && poz_slider >= lower_limit_slider) {
                    motor_slider.setPower(put_slider);
                } else
                    motor_slider.setPower(0);
            }

            if (logs) {
                telemetry.addData("Pozitie Curenta Motor Slider:", poz_slider);
                telemetry.addData("Putere Teoretica Motor Slider:", put_slider);
                telemetry.addData("Putere Practica Motor Slider:", motor_slider.getPower());
            }

        }


        //Motor carusel si variabila toggle
        private boolean carusel_tgl = false;
        private boolean carusel_last_it = false;
        private boolean carusel_curr_it = false;
        public void carusel() {

            carusel_last_it = carusel_curr_it;
            carusel_curr_it = gamepad1.x;

            if (carusel_curr_it && !carusel_last_it)
                carusel_tgl = !carusel_tgl;

            if (carusel_tgl)
                motor_carusel.setPower(1);
            else
                motor_carusel.setPower(0);

        }

        //Functie pentru colector
        private int clear_colector = 200;
        boolean last_it_collector = false;
        boolean curr_it_collector = false;
        boolean tgl_collector = false;
        public void colector() {
            last_it_collector = curr_it_collector;
            curr_it_collector = gamepad2.y;
            if (curr_it_collector && !last_it_collector)
                tgl_collector = !tgl_collector;

            if (tgl_collector)
                motor_colector.setPower(1);
            else
                motor_colector.setPower(0);

        }

        //Functie pentru servo galetusa si distanta de la care se ridica automat servo-ul
        private int clear_holder = 2950;
        public void holder_cr(boolean logs) {
            if (gamepad1.left_bumper && poz_slider >= clear_holder)
                holder.setPower(1);
            else if (poz_slider >= clear_holder)
                holder.setPower(-0.7);
            else
                holder.setPower(-1);

        }
    }
}


