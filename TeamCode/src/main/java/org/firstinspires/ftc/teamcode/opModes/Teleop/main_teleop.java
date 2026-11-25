package org.firstinspires.ftc.teamcode.opModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class main_teleop extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor frontLeftDrive = null;

    private DcMotorEx barrel = null;
    private DcMotorEx barreltwo = null;

    private DcMotor intake = null;
    private DcMotor intaketwo = null;

    // ---------------- Barrel shooter RPM settings ----------------
    // Encoder ticks per revolution of the motor's OUTPUT SHAFT.
    //   goBILDA 5203 6000 RPM (1:1)   = 28
    //   goBILDA 5203 1620 RPM (3.7:1) = 103.8
    //   REV HD Hex bare (no gearbox)  = 28
    // CHANGE THIS to match your barrel motors.
    static final double BARREL_TICKS_PER_REV = 28.0;

    // Your old power range 0.5–0.8 is roughly 3000–4800 RPM on a 6000 RPM motor.
    static final double BARREL_RPM_INIT      = 4000;
    static final double BARREL_RPM_INCREMENT = 250;
    static final double BARREL_RPM_MIN       = 3000;
    static final double BARREL_RPM_MAX       = 5000;
    static final double BARREL_RPM_TOLERANCE = 100;   // for the "AT SPEED" indicator
    // --------------------------------------------------------------

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        barrel = hardwareMap.get(DcMotorEx.class, "barrel_motor");
        intake = hardwareMap.get(DcMotor.class, "intake_motor");
        barreltwo = hardwareMap.get(DcMotorEx.class, "barreltwo_motor");
        intaketwo = hardwareMap.get(DcMotor.class, "intaketwo_motor");

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        barrel.setDirection(DcMotor.Direction.REVERSE);
        barreltwo.setDirection(DcMotor.Direction.FORWARD);

        // Velocity control needs encoders.
        barrel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        barreltwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        barrel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        barreltwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Let the flywheels coast down instead of braking hard.
        barrel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        barreltwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Optional: custom PIDF for faster spin-up / less drop per shot (tune later).
        // barrel.setVelocityPIDFCoefficients(10, 3, 0, 12);
        // barreltwo.setVelocityPIDFCoefficients(10, 3, 0, 12);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double rf =0, lf =0, rr=0, lr=0;

        // debounce for barrel RPM increment/decrement
        boolean circle_last = false;
        boolean triangle_last = false;
        double debounceDelay = 50; // milliseconds
        double last_triangle_time = 0;
        double last_circle_time = 0;

        double barrelRPM = BARREL_RPM_INIT;
        boolean barrelOn = false;
        double max;

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize drive motor power so max is 1
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftDrive.setPower(frontLeftPower + lf);
            frontRightDrive.setPower(frontRightPower + rf);
            backLeftDrive.setPower(backLeftPower + lr);
            backRightDrive.setPower(backRightPower + rr);

            // Gamepad 2:
            // Square: Barrel on
            // Cross: Barrel off
            // Triangle: Increase barrel RPM
            // Circle: Decrease barrel RPM
            //
            // Left bumper:   Intake on
            // Right bumper:  Intake off
            // Right trigger: Intake reverse
            //
            // Dpad down: Intake 2 on
            // Dpad up:   Intake 2 off
            // Dpad left: Intake 2 reverse

            boolean triangle_state = gamepad2.triangle;
            boolean circle_state   = gamepad2.circle;

            double now = System.currentTimeMillis();

            if ((triangle_state && !triangle_last) &&
                    ( now - last_triangle_time) > debounceDelay ) {
                barrelRPM += BARREL_RPM_INCREMENT;
                last_triangle_time = now;
            }
            if ((circle_state && !circle_last) &&
                    ( now - last_circle_time) > debounceDelay ) {
                barrelRPM -= BARREL_RPM_INCREMENT;
                last_circle_time = now;
            }

            barrelRPM = Math.max(BARREL_RPM_MIN, Math.min(BARREL_RPM_MAX, barrelRPM));

            triangle_last = triangle_state;
            circle_last   = circle_state;

            if (gamepad2.square) {
                barrelOn = true;
            } else if (gamepad2.cross) {
                barrelOn = false;
            }

            // Apply every loop so RPM changes take effect immediately.
            if (barrelOn) {
                setBarrelRPM(barrelRPM);
            } else {
                setBarrelRPM(0);
            }

            if (gamepad2.left_bumper) {
                intake.setPower(1);
            }

            if (gamepad2.right_bumper) {
                intake.setPower(0);
            }

            if (gamepad2.right_trigger_pressed) {
                intake.setPower(-1);
            }

            if (gamepad2.dpad_down) {
                intaketwo.setPower(1);
                intake.setPower(1);
            }

            if (gamepad2.dpad_up) {
                intaketwo.setPower(0);
            }

            if (gamepad2.dpad_left) {
                intaketwo.setPower(-1);
            }

            // Shooter telemetry
            double rpm1 = getRPM(barrel);
            double rpm2 = getRPM(barreltwo);
            boolean atSpeed = barrelOn
                    && Math.abs(rpm1 - barrelRPM) <= BARREL_RPM_TOLERANCE
                    && Math.abs(rpm2 - barrelRPM) <= BARREL_RPM_TOLERANCE;

            telemetry.addData("Barrel", barrelOn ? "ON" : "OFF");
            telemetry.addData("Target RPM", "%.0f", barrelRPM);
            telemetry.addData("Barrel RPM", "%.0f", rpm1);
            telemetry.addData("BarrelTwo RPM", "%.0f", rpm2);
            telemetry.addData("Ready to shoot", atSpeed ? "AT SPEED" : "spinning up...");
            telemetry.update();
        }
    }

    // ===================== Barrel RPM helpers =====================

    private void setBarrelRPM(double rpm) {
        double ticksPerSec = rpm * BARREL_TICKS_PER_REV / 60.0;
        barrel.setVelocity(ticksPerSec);
        barreltwo.setVelocity(ticksPerSec);
    }

    private double getRPM(DcMotorEx motor) {
        return motor.getVelocity() * 60.0 / BARREL_TICKS_PER_REV;
    }
}