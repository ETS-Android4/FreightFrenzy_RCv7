package org.firstinspires.ftc.teamcode.Main;


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

import java.nio.file.attribute.FileOwnerAttributeView;
import java.util.function.Function;

@TeleOp(name = "Brat1", group = "main")

public class Driving1 extends LinearOpMode {


    private DcMotorEx motor_brat;
    private DcMotorEx motor_slider;
    private DcMotorEx motor_carusel;
    private DcMotorEx motor_colector;
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;


    private Servo holder;

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

        //Init pentru motoarele de la roti
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //Init pentru servo-ul de la holder de elemente
        holder = hardwareMap.servo.get("holder");
        holder.setDirection(Servo.Direction.FORWARD);
        holder.setPosition(0.1);

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Apasa pe buton!", "");
            telemetry.update();

        }


        while (opModeIsActive())
        {
            fx.brat(true);
            fx.slider(true);
            fx.carusel();
            fx.colector();
            fx.holder_cr(false);
            fx.smoothMovement(true);
            if (gamepad1.b) {
                fx.reset_brat();
                sleep(fx.button_sleep);
            }

            telemetry.update();
        }

    }

    class functions {
        private int button_sleep = 75;

        private int upper_limit_brat = -130;
        private int lower_limit_brat = 3060;
        private boolean clear_brat = false;

        public void brat(boolean logs) {
            if (poz_slider * -1 >= 1000)
                clear_brat = true;
            else
                clear_brat = false;

            if (gamepad1.dpad_down && motor_brat.getCurrentPosition() <= lower_limit_brat && clear_brat)
                motor_brat.setPower(0.75);
            else if (gamepad1.dpad_up && motor_brat.getCurrentPosition() >= upper_limit_brat && clear_brat)
                motor_brat.setPower(-0.75);
            else
                motor_brat.setPower(0);

            if (logs) {
                telemetry.addData("Pozitie Curenta Motor Brat: ", motor_brat.getCurrentPosition());
                telemetry.addData("Putere Teoretica Motor Brat: ", 0.75);
                telemetry.addData("Putere Practica Motor Brat: ", motor_brat.getPower());
            }
        }

        public void reset_brat() {
            int semn = 1;

            while (motor_brat.getCurrentPosition() >= 20 || motor_brat.getCurrentPosition() <= -20) {
                if (motor_brat.getCurrentPosition() >= 0)
                    semn = -1;
                motor_brat.setPower(0.5 * semn);
            }
        }

        private int poz_slider = 0;
        private int upper_limit_slider = 5450;
        private int lower_limit_slider = 5;

        public void slider(boolean logs) {
            poz_slider = motor_slider.getCurrentPosition();
            if (gamepad1.right_trigger >= 0.3 && poz_slider * -1 <= upper_limit_slider) {
                motor_slider.setPower(-1);
            } else if (gamepad1.left_trigger >= 0.3 && poz_slider * -1 >= lower_limit_slider) {
                motor_slider.setPower(1);
            } else {
                motor_slider.setPower(0);
            }

            if (logs) {
                telemetry.addData("Pozitie Curenta Motor Slider:", motor_slider.getCurrentPosition() * -1);
                telemetry.addData("Putere Teoretica Motor Slider:", 0.4);
                telemetry.addData("Putere Practica Motor Slider:", motor_slider.getPower());
            }

        }

        public void carusel() {

            if (gamepad1.x && motor_carusel.getPower() <= 0.5) {
                motor_carusel.setPower(1);
                sleep(button_sleep);
            } else if (gamepad1.x && motor_carusel.getPower() >= 0.5) {
                motor_carusel.setPower(0);
                sleep(button_sleep);
            }

        }

        public void colector() {
            if (gamepad1.y && motor_colector.getPower() == 0) {
                motor_colector.setPower(1);
                sleep(button_sleep);
            } else if (gamepad1.y && motor_colector.getPower() == 1) {
                motor_colector.setPower(0);
                sleep(button_sleep);
            }
        }

        public void holder_cr(boolean logs) {
            if (gamepad1.left_bumper)
                holder.setPosition(1);

            else
                holder.setPosition(0.1);

        }

        int viteza = 0;
        double power = 0.8;
        double halfPower = 0.5;

        public void smoothMovement(boolean getLogs) {

            if (getLogs) {
                telemetry.addData("~~~~~~~~~~~~~~~~~~~~", "");
                telemetry.addData("Front left ", frontLeft.getCurrentPosition());
                telemetry.addData("Front right ", frontRight.getCurrentPosition());
                telemetry.addData("Back left ", backLeft.getCurrentPosition());
                telemetry.addData("Back right ", backRight.getCurrentPosition());
                telemetry.addData("~~~~~~~~~~~~~~~~~~~~", "");
            }

            if (gamepad2.right_bumper && viteza == 0) {
                viteza = 1;
                sleep(200);
            } else if (gamepad2.right_bumper && viteza == 1) {
                viteza = 0;
                sleep(200);
            }

            if (viteza == 0) {
                if (gamepad2.dpad_up) {
                    frontLeft.setPower(power + gamepad2.right_stick_x / 2);
                    frontRight.setPower(power - gamepad2.right_stick_x / 2);
                    backRight.setPower(power - gamepad2.right_stick_x / 2);
                    backLeft.setPower(power + gamepad2.right_stick_x / 2);
                } else if (gamepad2.dpad_down) {
                    frontLeft.setPower(-power + gamepad2.right_stick_x / 2);
                    frontRight.setPower(-power - gamepad2.right_stick_x / 2);
                    backRight.setPower(-power - gamepad2.right_stick_x / 2);
                    backLeft.setPower(-power + gamepad2.right_stick_x / 2);
                } else if (gamepad2.dpad_right) {
                    frontLeft.setPower(power + gamepad2.right_stick_x / 2);
                    frontRight.setPower(-power - gamepad2.right_stick_x / 2);
                    backRight.setPower(power - gamepad2.right_stick_x / 2);
                    backLeft.setPower(-power + gamepad2.right_stick_x / 2);
                } else if (gamepad2.dpad_left) {
                    frontLeft.setPower(-power + gamepad2.right_stick_x / 2);
                    frontRight.setPower(power - gamepad2.right_stick_x / 2);
                    backRight.setPower(-power - gamepad2.right_stick_x / 2);
                    backLeft.setPower(power + gamepad2.right_stick_x / 2);
                } else {
                    frontLeft.setPower(+gamepad2.right_stick_x + frontLeft.getPower() / 4);
                    frontRight.setPower(-gamepad2.right_stick_x + frontRight.getPower() / 4);
                    backRight.setPower(-gamepad2.right_stick_x + backRight.getPower() / 4);
                    backLeft.setPower(+gamepad2.right_stick_x + backLeft.getPower() / 4);
                }
            } else if (viteza == 1) {
                if (gamepad2.dpad_up) {
                    frontLeft.setPower(halfPower + gamepad2.right_stick_x / 2);
                    frontRight.setPower(halfPower - gamepad2.right_stick_x / 2);
                    backRight.setPower(halfPower - gamepad2.right_stick_x / 2);
                    backLeft.setPower(halfPower + gamepad2.right_stick_x / 2);
                } else if (gamepad2.dpad_down) {
                    frontLeft.setPower(-halfPower + gamepad2.right_stick_x / 2);
                    frontRight.setPower(-halfPower - gamepad2.right_stick_x / 2);
                    backRight.setPower(-halfPower - gamepad2.right_stick_x / 2);
                    backLeft.setPower(-halfPower + gamepad2.right_stick_x / 2);
                } else if (gamepad2.dpad_right) {
                    frontLeft.setPower(halfPower + gamepad2.right_stick_x / 2);
                    frontRight.setPower(-halfPower - gamepad2.right_stick_x / 2);
                    backRight.setPower(halfPower - gamepad2.right_stick_x / 2);
                    backLeft.setPower(-halfPower + gamepad2.right_stick_x / 2);
                } else if (gamepad2.dpad_left) {
                    frontLeft.setPower(-halfPower + gamepad2.right_stick_x / 2);
                    frontRight.setPower(halfPower - gamepad2.right_stick_x / 2);
                    backRight.setPower(-halfPower - gamepad2.right_stick_x / 2);
                    backLeft.setPower(halfPower + gamepad2.right_stick_x / 2);
                } else {
                    frontLeft.setPower(+gamepad2.right_stick_x / 2 + frontLeft.getPower() / 2);
                    frontRight.setPower(-gamepad2.right_stick_x / 2 + frontRight.getPower() / 2);
                    backRight.setPower(-gamepad2.right_stick_x / 2 + backRight.getPower() / 2);
                    backLeft.setPower(+gamepad2.right_stick_x / 2 + backLeft.getPower() / 2);
                }
            }
        }
    }
}


