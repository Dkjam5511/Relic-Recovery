package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.robotcore.internal.AppUtil;

/**
 * Created by Drew on 10/16/2016.
 * :P
 */
@TeleOp(name="TeleOp", group="Drive")
public class TeleOP_2016 extends OpMode {

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor LiftMotor;
    DcMotor ShootMotor;
    DcMotor SweepMotor;
    Servo btn_servo;
    Servo left_fork_servo;
    Servo right_fork_servo;
    Servo ball_gate_servo;
    Servo ball_loader;
    Servo fork_leveler;
    DeviceInterfaceModule CDI;
    double init_btn_servo_position = .45;
    double btn_servo_degrees = .2;
    double leftWheelPower = 0;
    double rightWheelPower = 0;
    double Speed = 1;
    double LiftSpeed = 1;
    double LeftSpeedInput;
    double RightSpeedInput;
    double xSpeedAdjustment;
    double RightStick_x;
    double LeftStick_y;
    double LeftStick_x;
    double ReductionFactor;
    double dpad_speed = .16;
    double dpad_turn_speed = .22;
    double fork_servo_power;
    double previous_fork_servo_power = .51;
    double LiftPower;
    double open_position = .4;
    double gate_closed_position = .9;
    double loader_down_position = .10;
    double loader_up_position = 1;
    double SM_start_position;
    double leveler_power;
    double previous_leveler_power = 0.5;
    boolean SlowMode = false;
    boolean joystick_driving = true;
    boolean BlueOn;
    boolean RedOn;
    boolean Reverse = false;
    boolean gate_open = false;
    boolean loader_up = false;
    boolean sweeper_running = false;
    boolean shoot_motor_running = false;
    boolean start_pressed = false;


    private ElapsedTime gate_timer = new ElapsedTime();
    private ElapsedTime loader_timer = new ElapsedTime();
    private ElapsedTime back_pressed_timer = new ElapsedTime();
    private ElapsedTime start_pressed_timer = new ElapsedTime();

    @Override
    public void init() {
        //Setting Up Devices in the Hardware Map
        leftWheel = hardwareMap.dcMotor.get("left_drive");
        rightWheel = hardwareMap.dcMotor.get("right_drive");
        LiftMotor = hardwareMap.dcMotor.get("lift_motor");
        ShootMotor = hardwareMap.dcMotor.get("shoot_motor");
        SweepMotor = hardwareMap.dcMotor.get("sweep_motor");
        btn_servo = hardwareMap.servo.get("button_servo");
        left_fork_servo = hardwareMap.servo.get("left_fork");
        right_fork_servo = hardwareMap.servo.get("right_fork");
        ball_gate_servo = hardwareMap.servo.get("ball_gate");
        ball_loader = hardwareMap.servo.get("ball_loader");
        fork_leveler = hardwareMap.servo.get("fork_leveler");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        btn_servo.setPosition(init_btn_servo_position);
        right_fork_servo.setDirection(Servo.Direction.REVERSE);

        right_fork_servo.setPosition(.5);
        left_fork_servo.setPosition(.49);

        ball_loader.setPosition(loader_down_position);

        ball_gate_servo.setPosition(gate_closed_position);

        fork_leveler.setPosition(previous_leveler_power + .045);

        ShootMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_pressed_timer.reset();

        start_pressed_timer.reset();
    }

    @Override
    public void start() {

    }


    @Override
    public void loop() {

        if (gamepad1.back && back_pressed_timer.seconds() > 0.3) {
            joystick_driving = !joystick_driving;
            back_pressed_timer.reset();
            telemetry.addData("Joystick Driving", joystick_driving);
            telemetry.update();
        }

        if (gamepad1.left_bumper) {
            SlowMode = false;
            BlueOn = false;
            RedOn = true;
        } else if (gamepad1.right_bumper) {
            SlowMode = true;
            BlueOn = true;
            RedOn = false;
        }

        if (SlowMode) {
            Speed = .25;
        } else {
            Speed = 1;
        }

        if (gamepad1.a) {
            Reverse = true;
        }

        if (gamepad1.b) {
            Reverse = false;
        }

        if (joystick_driving) {
            LeftStick_y = gamepad1.left_stick_y * Speed;
            LeftStick_x = gamepad1.left_stick_x * Speed;
            RightStick_x = gamepad1.right_stick_x * Speed;
            if (Math.abs(LeftStick_y) < .2 && Math.abs(LeftStick_x) > .2) {  //  Now we're in spinning mode
                LeftSpeedInput = -LeftStick_x;
                RightSpeedInput = LeftStick_x;
            } else {
                if (LeftStick_y > 0) {
                    RightStick_x = -RightStick_x;
                }
                LeftSpeedInput = LeftStick_y - (RightStick_x / 2.02);
                RightSpeedInput = LeftStick_y + (RightStick_x / 2.02);
                ReductionFactor = Math.max(Math.abs(LeftSpeedInput), Math.abs(RightSpeedInput)) - 1;
                if (ReductionFactor > 0) {
                    if (LeftStick_y < 0) {
                        ReductionFactor = -ReductionFactor;
                    }
                    LeftSpeedInput = LeftSpeedInput - ReductionFactor;
                    RightSpeedInput = RightSpeedInput - ReductionFactor;
                }
            }
        } else {
            LeftSpeedInput = gamepad1.left_stick_y * Speed;
            RightSpeedInput = gamepad1.right_stick_y * Speed;
        }

        /*
        if (joystick_driving) {
            LeftSpeedInput = gamepad1.left_stick_y * Speed;
            xSpeedAdjustment = gamepad1.left_stick_x * Speed;
            if (LeftSpeedInput == 0 && xSpeedAdjustment == 0) {
                LeftSpeedInput = gamepad1.right_stick_y * Speed; // if left stick isn't used, try right stick
                xSpeedAdjustment = gamepad1.right_stick_x * Speed;
            }
            RightSpeedInput = LeftSpeedInput;  // set right and left wheel to same
            if (LeftSpeedInput > .2) {  // this is so spinning doesn't switch to backing up and turning (which is the opposite direction of x) until joystick is pulled back past .2
                xSpeedAdjustment = -xSpeedAdjustment;
            }
            LeftSpeedInput = LeftSpeedInput - xSpeedAdjustment;
            RightSpeedInput = RightSpeedInput + xSpeedAdjustment;
            xSpeedAdjustment = 1;  // now use xSpeedAdjustment as a percentage to cap speed at 1.
            if (Math.abs(LeftSpeedInput) > 1) {
                xSpeedAdjustment = 1 / Math.abs(LeftSpeedInput);
            } else if (Math.abs(RightSpeedInput) > 1) {
                xSpeedAdjustment = 1 / Math.abs(RightSpeedInput);
            }
            LeftSpeedInput = LeftSpeedInput * xSpeedAdjustment;
            RightSpeedInput = RightSpeedInput * xSpeedAdjustment;
        } else {
            LeftSpeedInput = gamepad1.left_stick_y * Speed;
            RightSpeedInput = gamepad1.right_stick_y * Speed;
        }
        */

        telemetry.addData("LeftSpeed", LeftSpeedInput);
        telemetry.addData("RightSpeed", RightSpeedInput);
        telemetry.addLine("<================>");
        telemetry.addData("Joystick Y", gamepad1.right_stick_y);
        telemetry.addData("Joystick X", gamepad1.right_stick_x);
        telemetry.addLine("<================>");
        telemetry.addData("Joystick Driving", joystick_driving);
        telemetry.update();


        if(gamepad1.dpad_up){
            LeftSpeedInput = -dpad_speed;
            RightSpeedInput = -dpad_speed;
        } else if (gamepad1.dpad_down){
            LeftSpeedInput = dpad_speed;
            RightSpeedInput = dpad_speed;
        } else if (gamepad1.dpad_left){
            LeftSpeedInput = dpad_turn_speed;
            RightSpeedInput = -dpad_turn_speed;
        } else if (gamepad1.dpad_right){
            LeftSpeedInput = -dpad_turn_speed;
            RightSpeedInput = dpad_turn_speed;
        }

        if (Reverse) {
            leftWheel.setDirection(DcMotor.Direction.REVERSE);
            rightWheel.setDirection(DcMotor.Direction.FORWARD);
            leftWheelPower = RightSpeedInput;
            rightWheelPower = LeftSpeedInput;
        } else {
            leftWheel.setDirection(DcMotor.Direction.FORWARD);
            rightWheel.setDirection(DcMotor.Direction.REVERSE);
            leftWheelPower = LeftSpeedInput;
            rightWheelPower = RightSpeedInput;
        }

        if(gamepad2.x){
            ShootMotor.setPower(1);
            shoot_motor_running = true;
        }

        // mod 2880 is the amount of ticks past the start point.  Shooter doesn't stop until it's all the way around (or at least to 2700)
        // This depends on the person making sure the shoot motor is at the start position before init
        if (shoot_motor_running && ShootMotor.getCurrentPosition() % 2880 > 2700) {
            ShootMotor.setPower(0);
            shoot_motor_running = false;
        }

        //Sending Those Wheel Powers to the Actual Wheels
        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);

        //Setting the btn_servo position to the triggers and Y to reset
        if (gamepad1.right_trigger == 1) {
            btn_servo.setPosition(init_btn_servo_position - btn_servo_degrees);
        }
        if (gamepad1.left_trigger == 1) {
            btn_servo.setPosition(init_btn_servo_position + btn_servo_degrees);
        }
        if (gamepad1.y) {
            btn_servo.setPosition(init_btn_servo_position);
        }



        // gamepad stick give us -1 to 1.  divide by 2 and add .5 to make it between 0 and 1 for 360 servos
        // .5 is the middle which makes servos supposed to be stopped
        fork_servo_power = gamepad2.right_stick_y / 2 + .5;
        if (fork_servo_power != previous_fork_servo_power) {
            right_fork_servo.setPosition(fork_servo_power);
            left_fork_servo.setPosition(fork_servo_power - .01);  // the .01 and .02 are adjustments because our servos don't work right and aren't stopped unless at .51 and .52
            previous_fork_servo_power = fork_servo_power;
        }

        if (gamepad2.right_bumper){
            LiftSpeed = .3;  // slow mode for lift
        }

        if (gamepad2.left_bumper){
            LiftSpeed = 1;
        }

        LiftPower = gamepad2.left_stick_y * LiftSpeed;
        LiftMotor.setPower(LiftPower);

        if (gamepad2.a) {
            gate_timer.reset();
            ball_gate_servo.setPosition(open_position);
            gate_open = true;
        }
        if (gate_open && gate_timer.milliseconds() > 140) {
            ball_gate_servo.setPosition(gate_closed_position);
            gate_open = false;
        }
        
        if (gamepad1.start) {
            SweepMotor.setPower(-1);
            ball_loader.setPosition(loader_down_position);
            loader_timer.reset();
            sweeper_running = true;
        }

        if (sweeper_running && loader_timer.seconds() <= 0.6 && !loader_up) {
            ball_loader.setPosition(loader_up_position);
            loader_up = true;
        } else if (sweeper_running && loader_timer.seconds() > 0.6 && loader_timer.seconds() < 2.1 && loader_up) {
            ball_loader.setPosition(loader_down_position);
            loader_up = false;
        } else if (loader_timer.seconds() >= 2.1) {
            loader_timer.reset();
        }

        if (gamepad1.x) {
            sweeper_running = false;
            SweepMotor.setPower(0);
            ball_loader.setPosition(loader_down_position);
        }

        if (gamepad2.start && !start_pressed && start_pressed_timer.seconds() > .25) {
            SweepMotor.setPower(-1);
            start_pressed = true;
            start_pressed_timer.reset();
        }

        if (gamepad2.start && start_pressed && start_pressed_timer.seconds() > .25) {
            SweepMotor.setPower(1);
            start_pressed = false;
            start_pressed_timer.reset();
        }

        if (gamepad2.dpad_up) {
            leveler_power = 1;
        }else if (gamepad2.dpad_down) {
            leveler_power = 0;
        } else{
            leveler_power = 0.545;
        }

        if (previous_leveler_power != leveler_power){
            fork_leveler.setPosition(leveler_power);
            previous_leveler_power = leveler_power;
        }


        /*
        if (gamepad2.b) {
            loader_timer.reset();
            ball_loader.setPosition(loader_up_position);
            loader_up = true;
        }
        if (loader_up && loader_timer.milliseconds() > 700) {
            ball_loader.setPosition(loader_down_position);
            loader_up = false;
        }
        */
        //Changing light based on speed
        CDI.setLED(0, BlueOn);           //Blue light
        CDI.setLED(1, RedOn);           //Red light


    }


    @Override
    public void stop() {

    }

}
