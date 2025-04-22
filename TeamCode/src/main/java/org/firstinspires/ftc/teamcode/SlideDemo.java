package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="SlideDemo", group="Tutorial")
public class SlideDemo extends LinearOpMode {
    // Constants for your belt drive:
    //   • GT2 pulley: 60 teeth × 2 mm/tooth = 120 mm per revolution :contentReference[oaicite:0]{index=0}&#8203;:contentReference[oaicite:1]{index=1}
    //   • Encoder CPR (Yellow Jacket gearbox + encoder): usually 537.6 ticks per output rev
    //     (check your motor datasheet).
    static final double TICKS_PER_REV   = 537.6;
    static final double MM_PER_REV      = 120.0;
    static final double TICKS_PER_MM    = TICKS_PER_REV / MM_PER_REV;

    private DcMotorEx slide;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1) Map and configure
        slide = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slide.setDirection(DcMotorSimple.Direction.FORWARD);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // — Manual jog with left stick up/down —
            double power = -gamepad1.left_stick_y;
            slide.setPower(power);

            // — Or press buttons to go to preset heights —
            if (gamepad1.a) { moveToMm(100.0,  0.8); }  // extend 100 mm
            if (gamepad1.b) { moveToMm(200.0, 0.8); }  // extend 200 mm
            if (gamepad1.y) { moveToMm(336.0, 0.8); }  // full single‑stage length

            telemetry.addData("pos (mm)", slide.getCurrentPosition() / TICKS_PER_MM);
            telemetry.update();
        }
    }

    /**
     * Move the slide to the given linear position (in millimeters) at the given power.
     */
    private void moveToMm(double mm, double power) {
        int targetTicks = (int)Math.round(mm * TICKS_PER_MM);
        slide.setTargetPosition(targetTicks);
        slide.setPower(Math.abs(power));
    }
}

