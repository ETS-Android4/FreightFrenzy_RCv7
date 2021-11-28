package org.firstinspires.ftc.teamcode.Main;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Function;

@TeleOp(name = "Brat1", group = "main")

public class Driving1 extends LinearOpMode {


    private DcMotorEx motor_brat;
    private functions fx = new functions();


    public void runOpMode() throws InterruptedException
    {
        motor_brat = hardwareMap.get(DcMotorEx.class, "brat");

        motor_brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_brat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_brat.setDirection(DcMotorSimple.Direction.REVERSE);

        while (!opModeIsActive() && !isStopRequested())
        {
            telemetry.addData("Apasa pe buton!", "");
            telemetry.update();

        }


        while (opModeIsActive())
        {
            fx.Brat(true);
            if (gamepad1.b) {
                fx.Reset_Brat();
                sleep(75);
            }
        }

    }

    class functions
    {
        private int upper_limit_brat = -130;
        private int lower_limit_brat = 3060;
        public void Brat(boolean logs)
        {
            if (gamepad1.dpad_down && motor_brat.getCurrentPosition() <= lower_limit_brat)
                motor_brat.setPower(0.75);
            else if (gamepad1.dpad_up && motor_brat.getCurrentPosition() >= upper_limit_brat)
                motor_brat.setPower(-0.75);
            else
                motor_brat.setPower(0);

            if (logs) {
                telemetry.addData("Pozitie curenta brat: ", motor_brat.getCurrentPosition());
                telemetry.addData("Putere Teoretica: ", 0.75);
                telemetry.addData("Putere Practica: ", motor_brat.getPower());
                telemetry.update();
            }
        }

        public void Reset_Brat()
        {
            int semn = 1;

            while(motor_brat.getCurrentPosition() >= 20 || motor_brat.getCurrentPosition() <= -20)
            {
                if (motor_brat.getCurrentPosition()>=0)
                    semn = -1;
                motor_brat.setPower(0.3*semn);
            }
        }

    }

}


