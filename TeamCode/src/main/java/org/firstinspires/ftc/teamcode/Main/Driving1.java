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

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;


import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import java.nio.file.attribute.FileOwnerAttributeView;
import java.util.function.Function;

@TeleOp(name = "Main_Driving", group = "main")

public class Driving1 extends LinearOpMode {


    private DcMotorEx motor_brat;
    private DcMotorEx motor_slider;
    private DcMotorEx motor_carusel;
    private DcMotorEx motor_colector;

    private int orientation_drive = -1;
    private SampleMecanumDrive mecanum_drive;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private CRServo holder;

    private functions fx = new functions();


    public void runOpMode() throws InterruptedException
    {
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

        //Init pentru motorul care se ocupa de caruselul cu rate
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

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Apasa pe buton!", "");
            telemetry.update();

        }


        while (opModeIsActive())
        {
            if (gamepad2.x && orientation_drive == -1)
            {
                orientation_drive = 1;
                sleep(fx.button_sleep);
            }
            else if (gamepad2.x && orientation_drive == 1)
            {
                orientation_drive = -1;
                sleep(fx.button_sleep);
            }

            mecanum_drive.setWeightedDrivePower(
                    new Pose2d(
                            orientation_drive*gamepad2.left_stick_y,
                            orientation_drive*gamepad2.left_stick_x,
                            -gamepad2.right_stick_x
                    )
            );
            mecanum_drive.update();

            fx.brat(true);
            fx.slider(true);
            fx.carusel();
            fx.colector();
            fx.holder_cr(false);
            fx.reset_brat();


            telemetry.update();
        }

    }

    class functions {
        private int button_sleep = 135;

        private boolean resetting_brat = false;

        private int upper_limit_brat = -130;
        private int lower_limit_brat = 2300;
        private boolean clear_brat = false;
        private int dist_min_ext = 900;
        private double put_brat = 0.8;

        public void brat(boolean logs) {
            if (poz_slider >= dist_min_ext)
                clear_brat = true;
            else
                clear_brat = false;

            if (poz_slider >= 1800)
                put_brat = 0.5;
            else
                put_brat = 0.8;

            if (resetting_brat == false) {
                if (gamepad1.dpad_down && motor_brat.getCurrentPosition() <= lower_limit_brat && clear_brat)
                    motor_brat.setPower(put_brat);
                else if (gamepad1.dpad_up && motor_brat.getCurrentPosition() >= upper_limit_brat && clear_brat)
                    motor_brat.setPower(-put_brat);
                else
                    motor_brat.setPower(0);
            }

            if (logs) {
                telemetry.addData("Pozitie Curenta Motor Brat: ", motor_brat.getCurrentPosition());
                telemetry.addData("Putere Teoretica Motor Brat: ", put_brat);
                telemetry.addData("Putere Practica Motor Brat: ", motor_brat.getPower());
            }
        }


        public void reset_brat() {
            if (gamepad1.b) {
                resetting_brat = true;
                motor_brat.setTargetPosition(0);
                motor_slider.setTargetPosition(0);

                motor_brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (motor_brat.isBusy())
                    motor_brat.setPower(1);

                if (motor_slider.isBusy())
                    motor_slider.setPower(1);

                sleep(button_sleep);
            }

            if (!motor_slider.isBusy() && !motor_brat.isBusy())
                resetting_brat = false;

            if (resetting_brat == false) {
                motor_brat.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }

        private int poz_slider = 0;
        private int upper_limit_slider = 5350;
        private int lower_limit_slider = 25;
        private double put_slider = 1;

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

        private boolean carusel_tgl = false;
        public void carusel() {

            if (gamepad1.x && !carusel_tgl) {
                motor_carusel.setPower(1);
                carusel_tgl = true;
                sleep(button_sleep);
            } else if (gamepad1.x && carusel_tgl) {
                motor_carusel.setPower(0);
                carusel_tgl = false;
                sleep(button_sleep);
            }

        }

        public void colector() {

            if (gamepad2.y && motor_colector.getPower() == 0 && poz_slider <= 100) {
                motor_colector.setPower(1);
                sleep(button_sleep);
            } else if (gamepad2.y && motor_colector.getPower() == 1) {
                motor_colector.setPower(0);
                sleep(button_sleep);
            }
        }


        private int clear_holder = 2950;
        public void holder_cr(boolean logs) {
            if (gamepad1.left_bumper && poz_slider >= clear_holder)
                holder.setPower(1);
            else if (poz_slider >= clear_holder)
                holder.setPower(-0.6);
            else
                holder.setPower(-1);

        }
    }
}


