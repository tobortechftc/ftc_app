/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * TobotHardware
 * <p/>
 * Define all hardware (e.g. motors, servos, sensors) used by Tobot
 */
public class TT_2016_Hardware extends LinearOpMode {

    // CONSTANT VALUES.
    final static double THRESHOLD = 0.1;
    final static double SERVO_SCALE = 0.001;
    final static double PUSHER_UP = 0.83;
    final static double PUSHER_DOWN_1 = 0.45;
    final static double PUSHER_UP1 = 0.75;
    final static double PUSHER_DOWN_2 = 0.35;
    final static double PUSHER_EXTRA = 0.1;
    final static double GATE_CLOSED = 0.1;
    final static double GATE_OPEN_SWEEPER_CLOSED = 0.4;
    final static double GATE_OPEN = 0.9;
    final static double GOLF_GATE_CLOSED = 0.54;
    final static double GOLF_GATE_OPEN = 0.92;
    final static double SLIDER_GATE_OPEN = 0.001;
    final static double SLIDER_GATE_CLOSED = 0.5;
    final static double LIGHT_SENSOR_UP = 0.03;
    final static double LIGHT_SENSOR_DOWN = 0.5;
    final static double LEFT_BEACON_PRESS = 0.50;
    final static double LEFT_BEACON_INIT = 0.17;
    final static double RIGHT_BEACON_PRESS = 0.43;
    final static double RIGHT_BEACON_INIT = 0.69;
    final static double LEFT_BEACON_SIDE_DOWN = 0.15;
    final static double LEFT_BEACON_SIDE_PRESS = 0.82;
    final static double LEFT_BEACON_SIDE_INIT = 0.15;
    final static double RIGHT_BEACON_SIDE_DOWN = 0.50;
    final static double RIGHT_BEACON_SIDE_PRESS = 0.01;
    final static double RIGHT_BEACON_SIDE_INIT = 0.55;
    final static double WHITE_MAX = 0.79;
    final static double WHITE_MIN = 0.55;
    final static double WHITE_OP = 0.08; // optical distance sensor white color number
    final static int WHITE_ADA = 9000;
    final static double WHITE_NXT = 1.9;
    final static double WHITE_ODS = 1.00;
    final static double RANGE_WALL = 186.2;
    final static double ULTRA_GOAL_MIN = 96;
    final static double ULTRA_GOAL_MAX = 101;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).
    static final int LED_CHANNEL = 1;

    final static int ONE_ROTATION = 1120; // for AndyMark-40 motor encoder one rotation

    // final static int ONE_ROTATION = 1680; // for AndyMark-60 motor encoder one rotation

    // final static double RROBOT = 11;  // number of wheel turns to get chassis 360-degree
    final static double RROBOT = 6.63;  // number of wheel turns to get chassis 360-degree turn
    final static double INCHES_PER_ROTATION = 12.57; // inches per chassis motor rotation based on 16/24 gear ratio
    final static double GYRO_ROTATION_RATIO_L = 0.65; // 0.83; // Ratio of Gyro Sensor Left turn to prevent overshooting the turn.
    final static double GYRO_ROTATION_RATIO_R = 0.65; // 0.84; // Ratio of Gyro Sensor Right turn to prevent overshooting the turn.
    final static double NAVX_ROTATION_RATIO_L = 0.75; // 0.84; // Ratio of NavX Sensor Right turn to prevent overshooting the turn.
    final static double NAVX_ROTATION_RATIO_R = 0.75; // 0.84; // Ratio of NavX Sensor Right turn to prevent overshooting the turn.
    // variables for sensors
      /* This is the port on the Core Device Interace Module */
  /* in which the navX-Micro is connected.  Modify this  */
  /* depending upon which I2C port you are using.        */
    private final int NAVX_DIM_I2C_PORT = 4;

    //
    // following variables are used by Arm/slider
    //
    int numOpLoops = 1;
    double armDelta = 0.1;
    int slider_counter = 0;
    boolean bPrevState = false;

    // position of servos
    boolean bCurrState = false;
    double light_sensor_sv_pos = 0;
    double left_beacon_sv_pos = 0;
    double right_beacon_sv_pos = 0;
    double gate_sv_pos = 0;
    double golf_gate_sv_pos = 0;
    double pusher_sv_pos = 0;
    //double left_beacon_side_sv_pos = 0;
    //double right_beacon_side_sv_pos = 0;
    // amount to change the claw servo position by
    double slider_gate_sv_pos = 0;
    Boolean blue_detected = false;
    Boolean red_detected = false;
    int detectwhite = 0;
    AHRS navx_device;
    BNO055IMU ada_imu; // The Adafruit IMU sensor object
    double yaw;
    ColorSensor coSensor; // right sensor
    ColorSensor coSensor2; // left sensor
    ColorSensor coAda;
    DeviceInterfaceModule cdim;
    TouchSensor tSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;
    UltrasonicSensor ultra;
    OpticalDistanceSensor opSensor;
    GyroSensor gyro;
    LightSensor lightSensor;
    OpticalDistanceSensor odsSensor;
    int heading = 360;
    double imu_heading = 0;
    int touch = 0;
    // LightSensor LL, LR;
    TT_ColorPicker colorPicker;

    // following variables are used by Chassis
    State state;
    Boolean use_navx = true;    // set to false to disable navx
    Boolean use_ada_imu = true; // set to false to disable adafruit IMU
    Boolean use_gyro = true;    // set to false to disable MR gyro
    Boolean use_encoder = true;
    Boolean use_ultra = true;
    Boolean use_range = true;
    Boolean use_adacolor = false;
    Boolean use_light = false;
    Boolean use_ods = true;
    Boolean shooting_range = false;
    float speedScale = (float) 0.7; // controlling the speed of the chassis in teleOp state
    float leftPower = 0;
    float rightPower = 0;
    float SW_power = 0;
    double SH_power = 0.95; // initial shooter-on power to 0.95
    double initAutoOpTime = 0;
    float currRaw = 0;
    DcMotor motorR;
    DcMotor motorL;
    DcMotor sweeper;
    DcMotor shooter;
    DcMotor linear_slider;
    Servo light_sensor_sv;
    Servo left_beacon_sv;
    Servo right_beacon_sv;
    Servo gate_sv;
    Servo golf_gate_sv;
    Servo pusher_sv;
    //Servo left_beacon_side_sv;
    //Servo right_beacon_side_sv;
    Servo slider_gate_sv;
    int motorRightCurrentEncoder = 0;
    int motorLeftCurrentEncoder = 0;
    int motorRightTargetEncoder = 0;
    int motorLeftTargetEncoder = 0;
    int leftCnt = 0; // left motor target counter
    int rightCnt = 0; // right motor target counter
    private boolean v_warning_generated = false;
    private String v_warning_message;

    public Servo init_servo(String name) {
        Servo new_sv;
        try {
            new_sv = hardwareMap.servo.get(name);
        } catch (Exception p_exeception) {
            m_warning_message(name);
            DbgLog.msg(p_exeception.getLocalizedMessage());
            new_sv = null;
        }
        DbgLog.msg(String.format("TOBOT init_servo() - %s", name));
        // commenting out following lines as the new release since March 2016,
        // Servo cannot get position right after initialization
        //double pos = new_sv.getPosition();
        //new_sv.setPosition(pos);
        return new_sv;
    }

    public void tobot_init(State st) throws InterruptedException {
        /*
         * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

        DbgLog.msg(String.format("TOBOT-INIT Begin - 8-27"));
        v_warning_generated = false;
        v_warning_message = "Can't map; ";

        if (st==State.STATE_TELEOP) { // disable IMU initialization
            use_navx = false;
            use_ada_imu = false;
            use_gyro = false;
        }

        //light_sensor_sv = init_servo("light_sensor_sv");
        left_beacon_sv = init_servo("left_beacon_sv");
        right_beacon_sv = init_servo("right_beacon_sv");
        //left_beacon_side_sv = init_servo("left_beacon_side_sv");
       // right_beacon_side_sv = init_servo("right_beacon_side_sv");
        gate_sv = init_servo("gate_sv");
        golf_gate_sv = init_servo("golf_sv");
        pusher_sv = init_servo("pusher_sv");
        slider_gate_sv = init_servo("slider_gate_sv");
        //DbgLog.msg(String.format("TOBOT-INIT  light_sensor_sv -"));
        //set_light_sensor(LIGHT_SENSOR_DOWN);
        set_left_beacon(LEFT_BEACON_INIT);
        set_right_beacon(RIGHT_BEACON_INIT);
        set_gate(GATE_CLOSED);
        set_golf_gate(GOLF_GATE_CLOSED);
        set_pusher(PUSHER_UP);
        set_slider_gate(SLIDER_GATE_CLOSED);


        long systemTime = System.nanoTime();


        // initialize chassis variables
        motorL = hardwareMap.dcMotor.get("motorL");
        motorR = hardwareMap.dcMotor.get("motorR");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        shooter = hardwareMap.dcMotor.get("shooter");
        linear_slider = hardwareMap.dcMotor.get("linear_slider");


        motorR.setDirection(DcMotor.Direction.REVERSE);
        motorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorL.setPower(0);
        motorR.setPower(0);
        sweeper.setPower(0);
        shooter.setPower(0);
        linear_slider.setPower(0);
        // shooter.setDirection(DcMotor.Direction.REVERSE);
        sweeper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        state = st;

        if (state == State.STATE_TELEOP) {

        } else { // tune up or Auto

        }

        if (state == State.STATE_AUTO || state == State.STATE_TELEOP) {
            set_drive_modes(DcMotor.RunMode.RUN_USING_ENCODER);
        } else { // State.STATE_TUNE
            set_drive_modes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        // initialize sensores
        cdim = hardwareMap.deviceInterfaceModule.get("dim");
        coSensor = hardwareMap.colorSensor.get("co");
        coSensor.setI2cAddress(I2cAddr.create8bit(0x60));


        coSensor2 = hardwareMap.colorSensor.get("co2");
        coSensor2.setI2cAddress(I2cAddr.create8bit(0x3c));
        coSensor2.enableLed(true);

        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        // set the digital channel to output mode.
        // remember, the Adafruit sensor is actually two devices.
        // It's an I2C sensor and it's also an LED that can be turned on or off.
        cdim.setDigitalChannelMode(LED_CHANNEL, DigitalChannelController.Mode.OUTPUT);

        // get a reference to our ColorSensor object.
        if (use_adacolor) {
            coAda = hardwareMap.colorSensor.get("color");
        }

        if (use_light) {
            lightSensor = hardwareMap.lightSensor.get("nxtLight");
            lightSensor.enableLed(true);
        }
        if (use_ods) {
            odsSensor = hardwareMap.opticalDistanceSensor.get("ods");
        }
        // bEnabled represents the state of the LED.
        boolean bEnabled = true;

        // turn the LED on in the beginning, just so user will know that the sensor is active.
        cdim.setDigitalChannelState(LED_CHANNEL, bEnabled);

        //tSensor = hardwareMap.touchSensor.get("to");
        //opSensor = hardwareMap.opticalDistanceSensor.get("op");
        if (use_ultra) {
            ultra = hardwareMap.ultrasonicSensor.get("ultra");
        }

        //LL = hardwareMap.lightSensor.get("ll");
        //LR = hardwareMap.lightSensor.get("lr");

        colorPicker = new TT_ColorPicker(coSensor, coSensor2); // right_color_sen, left_color_sen

        //Instantiate ToborTech Nav objects:
        // 1. navx
        // 2. ada imu
        // 3. gyro
        if (use_navx) {
            navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                    NAVX_DIM_I2C_PORT,
                    AHRS.DeviceDataType.kProcessedData);

            double init_time = getRuntime();
            boolean navx_ok = false;
            while (!navx_ok && (getRuntime() - init_time < 10)) { // wait for 6 sec to get connected
                navx_ok = navx_device.isConnected();
                idle();
            }
            if (navx_ok) {
                boolean navx_cal = true;
                while (navx_cal && (getRuntime() - init_time < 16)) { // wait for 12 sec to get calibration
                    navx_cal = navx_device.isCalibrating();
                    idle();
                }
                if (navx_cal) {
                    navx_ok = false;
                    DbgLog.msg(String.format("TOBOT-INIT: NaxX IMU is not calibrated properly!"));
                }
            } else {
                DbgLog.msg(String.format("TOBOT-INIT: NaxX IMU is not connected!"));
            }
            if (navx_ok) {
                navx_device.zeroYaw();
                sleep(500);
                double pre_yaw = navx_device.getYaw();
                driveTT(0.3, 0.25);
                sleep(5);
                driveTT(0, 0);
                sleep(10);
                imu_heading = navx_device.getYaw();
                if (Math.abs(imu_heading - pre_yaw) > 0.0001) {
                    use_navx = true;
                } else use_navx = false;
            } else {
                use_navx = false;
            }
        }

        if (!use_navx && use_ada_imu) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "ADA_IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            ada_imu = hardwareMap.get(BNO055IMU.class, "imu");
            if (ada_imu.initialize(parameters) == false) {
                use_ada_imu = false;
                DbgLog.msg(String.format("TOBOT-INIT: AdaFruit IMU is not initialized properly!"));
            }
        } else {
            use_ada_imu = false;
        }
        if (!use_navx && !use_ada_imu && use_gyro) {
            gyro = hardwareMap.gyroSensor.get("gyro");
            // calibrate the gyro.
            double init_time = getRuntime();
            gyro.calibrate();
            while (!isStopRequested() && gyro.isCalibrating() && (getRuntime() - init_time < 10)) {
                sleep(50);
                idle();
            }
            if (gyro.isCalibrating()) {
                use_gyro = false;
            }
        } else {
            use_gyro = false;
        }
        adjustShooterPower();
        hardwareMap.logDevices();
        stop_tobot();
        show_telemetry();
        DbgLog.msg(String.format("TOBOT-INIT  end() -"));
    } // end of tobot_init

    @Override
    public void runOpMode() throws InterruptedException {

        tobot_init(State.STATE_TELEOP);

        waitForStart();

        while (opModeIsActive()) {
            //show_telemetry();
        }
        stop_tobot();
    }

    double ada_imu_heading() {
        Orientation angles = ada_imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
        return (double) (angles.firstAngle);
    }

    public void show_telemetry() {
        double cur_heading = 0;
        if (state == State.STATE_TUNEUP) {
            if (use_navx) {
                cur_heading = navx_device.getYaw();
            } else if (use_ada_imu) {
                cur_heading = ada_imu_heading();
            } else if (use_gyro) {
                cur_heading = (double) (gyro.getHeading());
            }
        }
        if (state == State.STATE_TUNEUP) {
            telemetry.addData("0. State / Speed-scale / Volt / SH-Pw: ",
                    String.format("%s / %.3f / %.3f / %.2f", state.toString(), speedScale, getBatteryVoltage(), SH_power));
        } else {
            telemetry.addData("0. State / Speed-scale / SH-Pw: ", String.format("%s / %.3f / %.2f",
                    state.toString(), speedScale, SH_power));
        }
        telemetry.addData("1. use NavX/ use Ada-imu/ use Gyro:", String.format("%s / %s / %s",
                use_navx.toString(), use_ada_imu.toString(), use_gyro.toString()));

        if (state == State.STATE_TUNEUP) {
            double ul_val = 0.0, rg_val = 0.0;
            if (use_ultra)
                ul_val = ultra.getUltrasonicLevel();
            if (use_range)
                rg_val = rangeSensor.getDistance(DistanceUnit.CM);

            telemetry.addData("2. IMU_heading/Current heading:", String.format("%.4f/%.4f", imu_heading, cur_heading));
            if (use_ada_imu && use_gyro) {
                telemetry.addData("3. Gyro-heading / Ada-IMU err = ", String.format("%d / %s",
                        gyro.getHeading(), ada_imu.getSystemError().toString()));
            } else if (use_ada_imu) {
                telemetry.addData("3. Ada-IMU sys-st / err-st = ", String.format("%s / %s",
                        ada_imu.getSystemStatus().toString(), ada_imu.getSystemError().toString()));
            } else if (use_gyro) {
                telemetry.addData("3. Gryo heading = ", String.format("%d", gyro.getHeading()));
            }
            telemetry.addData("4. Color1 R/G/B  = ", String.format("%d / %d / %d", coSensor.red(), coSensor.green(), coSensor.blue()));
            telemetry.addData("5. Color2 R/G/B  = ", String.format("%d / %d / %d", coSensor2.red(), coSensor2.green(), coSensor2.blue()));
            telemetry.addData("6. White/range/ul(Goal)= ", String.format("%d/%.2f/%.2f(%s)", detectwhite, rg_val, ul_val, shooting_range.toString()));
            if (use_adacolor) {
                telemetry.addData("7. Ada C/B/R/G/Sum  = ", String.format("%d/%d/%d/%d/%d", coAda.alpha(), coAda.blue(), coAda.red(), coAda.green(),
                        (coAda.alpha() + coAda.blue() + coAda.red() + coAda.green())));
            }
            telemetry.addData("8. drive power: L=", String.format("%.2f", leftPower) + "/R=" + String.format("%.2f", rightPower));
            telemetry.addData("9. gate/pusher/golf= ", String.format("%.2f/%.2f/%.2f", gate_sv_pos, pusher_sv_pos, golf_gate_sv_pos));

            //telemetry.addData("10. sv ls/l_b/r_b  = ", String.format("%.2f / %.2f / %.2f", light_sensor_sv_pos, left_beacon_sv_pos, right_beacon_sv_pos));

            if (use_light) {
                telemetry.addData("11. light_s Raw/Norm", String.format("%.2f / %.2f",
                        lightSensor.getRawLightDetected(), lightSensor.getLightDetected()));
            }
            if (use_ods) {
                telemetry.addData("12. ODS Raw/Norm", String.format("%.2f / %.2f",
                        odsSensor.getRawLightDetected(), odsSensor.getLightDetected()));
            }
        }

        //telemetry.addData("7. left  cur/tg enc:", motorBL.getCurrentPosition() + "/" + leftCnt);
        //telemetry.addData("8. right cur/tg enc:", motorBR.getCurrentPosition() + "/" + rightCnt);
        //show_heading();
        telemetry.update();
        // Dbg.msg(String.format("Gyro heading tar/curr = %d/%d, power L/R = %.2f/%.2f",
        //                   heading, cur_heading, leftPower, rightPower));
    }

    public void show_heading() {
        touch = (tSensor.isPressed() ? 1 : 0);
        telemetry.addData("9. head/gyro/ods/touch:", String.format("%d/%d/%.4f/%d",
                heading, gyro.getHeading(), opSensor.getLightDetected(), touch));
        telemetry.addData("9. head/gyro/ods/ultra/touch:", String.format("%d/%d/%.4f/%.2f/%d",
                heading, gyro.getHeading(), opSensor.getLightDetected(), ultra.getUltrasonicLevel(), touch));
    }

    public void StraightR(double power, double n_rotations) throws InterruptedException {
        if (!opModeIsActive()) return;
        reset_chassis();
        // set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorL.getCurrentPosition();
        int rightEncode = motorR.getCurrentPosition();
        initAutoOpTime = this.time;
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftCnt = (int) (ONE_ROTATION * n_rotations);
        rightCnt = (int) (ONE_ROTATION * n_rotations);
        leftPower = rightPower = (float) power;
        if (power < 0) { // move backward
            leftCnt = leftEncode - leftCnt;
            rightCnt = rightEncode - rightCnt;
        } else {
            leftCnt += leftEncode;
            rightCnt += rightEncode;
        }
        run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);

        sleep(135);
    }

    public int mapHeading(int n) {
        if (n < 45) return (n + 360);
        return n;
    }

    public void StraightIn(double power, double in) throws InterruptedException {
        if (use_navx) {
            imu_heading = navx_device.getYaw();
        } else if (use_ada_imu) {
            imu_heading = ada_imu_heading();
        }
        if (use_encoder) {
            double numberR = in / INCHES_PER_ROTATION;
            StraightR(power, numberR);
        } else { // using timer
            double in_per_ms = 0.014 * power / 0.8;
            if (in_per_ms < 0) in_per_ms *= -1.0;
            long msec = (long) (in / in_per_ms);
            if (msec < 100) msec = 100;
            if (msec > 6000) msec = 6000; // limit to 6 sec
            driveTT(power, power);
            sleep(msec);
            driveTT(0, 0);
        }
    }

    public void driveTT(double lp, double rp) {
        //if (use_gyro == true && lp == rp) {
        if (use_navx) {
            double cur_heading = navx_device.getYaw();
            if (cur_heading - imu_heading > 0.7) { // crook to right,  slow down left motor
                if (lp > 0) lp *= 0.9;
                else rp *= 0.9; // adjust backward
            } else if (cur_heading - imu_heading < -0.7) { // crook to left, slow down right motor
                if (rp > 0) rp *= 0.9;
                else lp *= 0.9;
            }

        } else if (use_ada_imu) {
            double cur_heading = ada_imu_heading();
            if (cur_heading - imu_heading > 0.7) { // crook to left,  slow down right motor
                if (rp > 0) rp *= 0.9;
                else lp *= 0.9;
            } else if (cur_heading - imu_heading < -0.7) { // crook to right, slow down left motor
                if (lp > 0) lp *= 0.9;
                else rp *= 0.9;
            }
        }
        motorR.setPower(rp);
        motorL.setPower(lp);
    }

    public void run_until_encoder(int leftCnt, double leftPower, int rightCnt, double rightPower) throws InterruptedException {
        //motorFR.setTargetPosition(rightCnt);
        //motorBL.setTargetPosition(leftCnt);
        //motorBL.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //motorFR.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        //waitOneFullHardwareCycle();
        int leftTC1 = leftCnt;
        int rightTC1 = rightCnt;
        int leftTC2 = 0;
        int rightTC2 = 0;
        if (leftPower > 0.4 && leftTC1 > 600) {
            leftTC2 = 500;
            leftTC1 -= 500;
        }
        if (rightPower > 0.4 && rightTC1 > 600) {
            rightTC2 = 500;
            rightTC1 -= 500;
        }
        driveTT(leftPower, rightPower);
        initAutoOpTime = getRuntime();
        //while (motorFR.isBusy() || motorBL.isBusy()) {
        while (!have_drive_encoders_reached(leftTC1, rightTC1) && (getRuntime() - initAutoOpTime < 5) && opModeIsActive()) {
            driveTT(leftPower, rightPower);
            show_telemetry();
        }
        if (rightTC2 > 0 || leftTC2 > 0) {
            driveTT(0.2, 0.2);
            while (!have_drive_encoders_reached(leftTC2, rightTC2) && (getRuntime() - initAutoOpTime < 7) && opModeIsActive()) {
                driveTT(0.2, 0.2);
                // show_telemetry();
            }
        }
        stop_chassis();
        if (state == State.STATE_AUTO) {
            set_drive_modes(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            set_drive_modes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // waitOneFullHardwareCycle();
    }

    public void TurnLeftD(double power, float degree, boolean spotTurn) throws InterruptedException {
        if (!opModeIsActive()) return;
        double adjust_degree_gyro = GYRO_ROTATION_RATIO_L * (double) degree;
        double adjust_degree_navx = NAVX_ROTATION_RATIO_L * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        initAutoOpTime = getRuntime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorL.getCurrentPosition();
        int rightEncode = motorR.getCurrentPosition();
        if (spotTurn) { // use both motors for spot turn
            leftCnt = (int) (-ONE_ROTATION * RROBOT * degree / 720.0);
            rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 720.0);
            leftPower = (float) -power;
        } else { // swing turn. only use right motor
            leftCnt = 0;
            rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 360.0);
            leftPower = (float) 0;
        }
        leftCnt += leftEncode;
        rightCnt += rightEncode;
        rightPower = (float) power;
        if (use_navx) {
            current_pos = navx_device.getYaw();
            imu_heading = current_pos - adjust_degree_navx;
            if (imu_heading <= -180) {
                imu_heading += 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos <= 0)) {
                current_pos += 360;
            }
            while ((current_pos >= imu_heading) && ((getRuntime() - initAutoOpTime) < 5.0) && opModeIsActive()) {
                current_pos = navx_device.getYaw();
                if (heading_cross_zero && (current_pos <= 0)) {
                    current_pos += 360;
                }
                driveTT(leftPower, rightPower);
            }
        } else if (use_ada_imu) {
            current_pos = ada_imu_heading();
            imu_heading = current_pos + adjust_degree_navx;
            if (imu_heading >= 0) {
                imu_heading -= 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos >= -180)) {
                current_pos -= 360;
            }
            while ((current_pos <= imu_heading) && ((getRuntime() - initAutoOpTime) < 5.0) && opModeIsActive()) {
                current_pos = ada_imu_heading();
                if (heading_cross_zero && (current_pos >= -180)) {
                    current_pos -= 360;
                }
                driveTT(leftPower, rightPower);
            }
        } else if (use_gyro) {
            initAutoOpTime = getRuntime();
            int cur_heading = gyro.getHeading();
            heading = gyro.getHeading() - (int) adjust_degree_gyro;
            Boolean cross_zero = false;
            if (heading < 0) {
                heading += 360;
                cur_heading += 360;
                cross_zero = true;
            }
            DbgLog.msg(String.format("LOP: Left Turn %.2f degree: Gyro tar/curr heading = %d/%d",
                    degree, heading, gyro.getHeading()));
            int prev_heading = -1;

            while (cur_heading > heading && (getRuntime() - initAutoOpTime < 4) && opModeIsActive()) {
                driveTT(leftPower, rightPower);
                if (prev_heading != cur_heading) {
                    DbgLog.msg(String.format("LOP: Gyro heading tar/curr = %d/%d, power L/R = %.2f/%.2f",
                            heading, cur_heading, leftPower, rightPower));
                }
                prev_heading = cur_heading;
                cur_heading = gyro.getHeading();
                if (cross_zero == true) {
                    if (cur_heading < adjust_degree_gyro) {
                        cur_heading += 360;
                    } else {
                        cross_zero = false;
                    }
                }

                // show_heading();
            }
        } else { // use encoder
            run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
        }
        driveTT(0, 0);

        sleep(135);
    }

    public void TurnRightD(double power, float degree, boolean spotTurn) throws InterruptedException {
        if (!opModeIsActive()) return;
        double adjust_degree_gyro = GYRO_ROTATION_RATIO_R * (double) degree;
        double adjust_degree_navx = NAVX_ROTATION_RATIO_R * (double) degree;
        double current_pos = 0;
        boolean heading_cross_zero = false;
        initAutoOpTime = getRuntime();
        reset_chassis();
        //set_drive_modes(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorFR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        //motorBL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int leftEncode = motorL.getCurrentPosition();
        int rightEncode = motorR.getCurrentPosition();
        if (spotTurn) { // use both motors for spot turn
            leftCnt = (int) (ONE_ROTATION * RROBOT * degree / 720.0);
            rightCnt = (int) (-ONE_ROTATION * RROBOT * degree / 720.0);
            rightPower = (float) -power;
        } else { // swing turn. only use right motor
            leftCnt = 0;
            rightCnt = (int) (ONE_ROTATION * RROBOT * degree / 360.0);
            rightPower = (float) 0;
        }
        leftCnt += leftEncode;
        rightCnt += rightEncode;
        leftPower = (float) power;

        if (use_navx) {
            current_pos = navx_device.getYaw();
            imu_heading = current_pos + adjust_degree_navx;
            if (imu_heading >= 180) {
                imu_heading -= 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos >= 0)) {
                current_pos -= 360;
            }
            while ((current_pos <= imu_heading) && ((getRuntime() - initAutoOpTime) < 5.0) && opModeIsActive()) {
                current_pos = navx_device.getYaw();
                if (heading_cross_zero && (current_pos >= 0)) {
                    current_pos -= 360;
                }
                driveTT(leftPower, rightPower);
            }
        } else if (use_ada_imu) {
            current_pos = ada_imu_heading();
            imu_heading = current_pos - adjust_degree_navx;
            if (imu_heading <= -360) {
                imu_heading += 360;
                heading_cross_zero = true;
            }
            if (heading_cross_zero && (current_pos <= -180)) {
                current_pos += 360;
            }
            while ((current_pos >= imu_heading) && ((getRuntime() - initAutoOpTime) < 5.0) && opModeIsActive()) {
                current_pos = ada_imu_heading();
                if (heading_cross_zero && (current_pos <= -180)) {
                    current_pos += 360;
                }
                driveTT(leftPower, rightPower);
            }
        } else if (use_gyro) {
            // if (false) {
            initAutoOpTime = getRuntime();
            int cur_heading = gyro.getHeading();
            heading = cur_heading + (int) adjust_degree_gyro;
            int prev_heading = -1;
            int init_heading = cur_heading;
            DbgLog.msg(String.format("LOP: Right Turn %.2f degree: Gyro tar/curr heading = %d/%d",
                    degree, heading, gyro.getHeading()));

            while (cur_heading < heading && (getRuntime() - initAutoOpTime < 5) && opModeIsActive()) {
                driveTT(leftPower, rightPower);
                if (prev_heading != cur_heading) {
                    DbgLog.msg(String.format("LOP: Gyro heading tar/curr = %d/%d, power L/R = %.2f/%.2f",
                            heading, cur_heading, leftPower, rightPower));
                }
                prev_heading = cur_heading;
                cur_heading = gyro.getHeading();
                if (heading > 360 && init_heading > cur_heading) {
                    cur_heading += 360;
                }
                // show_heading();
            }
        } else {
            if (use_encoder) {
                run_until_encoder(leftCnt, leftPower, rightCnt, rightPower);
            } else {
                long degree_in_ms = 33 * (long) degree;
                driveTT(leftPower, rightPower);
                sleep(degree_in_ms);
                driveTT(0, 0);
            }

        }
        driveTT(0, 0);
        sleep(135);
    }

    void set_drive_modes(DcMotor.RunMode mode) {
        motorL.setMode(mode);
        motorR.setMode(mode);
    }

    void stop_chassis() {
        motorL.setPower(0);
        motorR.setPower(0);
    }

    void stop_tobot() {
        stop_chassis();
        SW_power = 0;
        sweeper.setPower(0);
        shooter.setPower(0);
        linear_slider.setPower(0);
    }

    void adjustShooterPower() {
        double cur_vol = getBatteryVoltage();
        // power = 1.0 when vol =< 13.0
        //         0.70 when vol >= 14.0
        // when cur_vol is >13
        //         power = 1.0 - 0.3 * (cur_vol - 13)
        if (cur_vol < 13.0) {
            SH_power = 1.0;
        } else {
            SH_power = ((1.0 - 0.3 * (cur_vol - 13))*1.0);
            if (SH_power > 1.0) {
                SH_power = 1.0;
            }
        }
    }

    void shootBallGateGolf() {
        set_gate(GATE_OPEN_SWEEPER_CLOSED);
        sleep(500);
        set_gate(GATE_CLOSED);
        set_golf_gate(GOLF_GATE_OPEN);
        sleep(400);
        set_golf_gate(GOLF_GATE_CLOSED);

    }
    void shootBallGolfGate(){
        set_golf_gate(GOLF_GATE_OPEN);
        set_pusher(PUSHER_DOWN_1);
        set_gate(GATE_OPEN_SWEEPER_CLOSED);
        sleep(300);
        set_pusher(PUSHER_DOWN_2);
        sleep(100);
        set_golf_gate(GOLF_GATE_CLOSED);
        sleep(300);
        set_gate(GATE_CLOSED);
        set_pusher(PUSHER_UP);
    }

    void shootAuto(boolean shoot_twice, double shooterPW){
        if (shooterPW > 1.0)
            shooterPW = 1.0;
        shooter.setPower(shooterPW);
        //push_ball();
        sleep(200);
        shootBallGolfGate();
        //sleep(500);
        //set_gate(GATE_CLOSED);
        if (shoot_twice && opModeIsActive()) {
            sleep(500);
            shooterPW = SH_power;
            if (shooterPW > 1.0)
                shooterPW = 1.0;
            shooter.setPower(shooterPW);
            //push_ball();

            //sleep(500);
            set_golf_gate(GOLF_GATE_OPEN);
            sleep(400);
            set_golf_gate(GOLF_GATE_CLOSED);
        }
        shooter.setPower(0.0);
    }


    // Computes the current battery voltage from ConceptTelemetry.java
    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    void reset_chassis() throws InterruptedException {
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (motorR.getCurrentPosition() != 0 && motorL.getCurrentPosition() != 0) {
            // && motorBR.getCurrentPosition()!=0) && motorFL.getCurrentPosition()!=0) {
            waitOneFullHardwareCycle();
        }
        leftCnt = 0;
        rightCnt = 0;
        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void reset_motors() throws InterruptedException {
        reset_chassis();
    }

    //--------------------------------------------------------------------------
    //
    // has_right_drive_encoder_reached
    //

    boolean has_left_drive_encoder_reached(double p_count) {
        if (leftPower < 0) {
            //return (Math.abs(motorFL.getCurrentPosition()) < p_count);
            return (motorL.getCurrentPosition() <= p_count);
        } else {
            //return (Math.abs(motorFL.getCurrentPosition()) > p_count);
            return (motorL.getCurrentPosition() >= p_count);
        }
    } // has_left_drive_encoder_reached

    //--------------------------------------------------------------------------
    //
    // have_drive_encoders_reached
    //

    /**
     * Indicate whether the right drive motor's encoder has reached a value.
     */
    boolean has_right_drive_encoder_reached(double p_count) {
        if (rightPower < 0) {
            return (motorR.getCurrentPosition() <= p_count);
        } else {
            return (motorR.getCurrentPosition() >= p_count);
        }

    } // has_right_drive_encoder_reached

    /**
     * Indicate whether the drive motors' encoders have reached specified values.
     */
    boolean have_drive_encoders_reached(double p_left_count, double p_right_count) {
        boolean l_return = false;
        if (has_left_drive_encoder_reached(p_left_count) && has_right_drive_encoder_reached(p_right_count)) {
            l_return = true;
        } else if (has_left_drive_encoder_reached(p_left_count)) { // shift target encoder value from right to left
            double diff = Math.abs(p_right_count - motorR.getCurrentPosition()) / 2;
            if (leftPower < 0) {
                leftCnt -= diff;
            } else {
                leftCnt += diff;
            }
            if (rightPower < 0) {
                rightCnt += diff;
            } else {
                rightCnt -= diff;
            }
        } else if (has_right_drive_encoder_reached(p_right_count)) { // shift target encoder value from left to right
            double diff = Math.abs(p_left_count - motorL.getCurrentPosition()) / 2;
            if (rightPower < 0) {
                rightCnt -= diff;
            } else {
                rightCnt += diff;
            }
            if (leftPower < 0) {
                leftCnt += diff;
            } else {
                leftCnt -= diff;
            }
        }
        return l_return;
    } // have_encoders_reached

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }

    void hit_right_button() throws InterruptedException {
        if (!opModeIsActive()) return;
        StraightIn(-0.3,0.5);
        set_right_beacon(RIGHT_BEACON_PRESS);
        sleep(330);
        bump_beacon();
        set_right_beacon(RIGHT_BEACON_INIT);
    }

    void bump_beacon() throws InterruptedException {
        if (!opModeIsActive()) return;
        driveTT(0.35, 0.35);
        sleep(550);
        driveTT(0, 0);
        if (!opModeIsActive()) return;
        StraightIn(-0.5, 3);
    }

    void hit_left_button() throws InterruptedException {
        if (!opModeIsActive()) return;
        StraightIn(-0.3,0.5);
        set_left_beacon(LEFT_BEACON_PRESS);
        sleep(330);
        bump_beacon();
        set_left_beacon(LEFT_BEACON_INIT);
    }

    public void forwardTillUltra(double us_stop_val, double power, double max_sec, boolean is_red) throws InterruptedException {
        double us_val = 0;
        if (is_red) {
            imu_heading = -45.0;
        } else {
            imu_heading = 45.0;
        }
        if (use_range) {
            // (true) {
            us_val = rangeSensor.getDistance(DistanceUnit.CM);
        }
        double init_time = getRuntime();
        while ((us_val > us_stop_val) && ((getRuntime() - init_time) < max_sec) && opModeIsActive()) {
            driveTT(power, power);
            // us_val = ultra.getUltrasonicLevel();
            if (use_range) {
                us_val = rangeSensor.getDistance(DistanceUnit.CM);
            }
        }
        driveTT(0, 0); // Make sure robot is stopped
    }

    public void set_light_sensor(double pos) {
        light_sensor_sv_pos = pos;
        light_sensor_sv.setPosition(light_sensor_sv_pos);
    }

    public void set_gate(double pos) {
        gate_sv_pos = pos;
        gate_sv.setPosition(gate_sv_pos);
    }

    public void set_golf_gate(double pos) {
        golf_gate_sv_pos = pos;
        golf_gate_sv.setPosition(golf_gate_sv_pos);
    }

    public void set_slider_gate(double pos) {
        slider_gate_sv_pos = pos;
        slider_gate_sv.setPosition(slider_gate_sv_pos);
    }

    public void set_left_beacon(double pos) {
        left_beacon_sv_pos = pos;
        left_beacon_sv.setPosition(left_beacon_sv_pos);
    }

    public void set_right_beacon(double pos) {
        right_beacon_sv_pos = pos;
        right_beacon_sv.setPosition(right_beacon_sv_pos);
    }

    public void set_pusher(double pos) {
        pusher_sv_pos = pos;
        pusher_sv.setPosition(pusher_sv_pos);
    }

    public void stopAtWhite(double power) throws InterruptedException {
        initAutoOpTime = getRuntime();
        driveTT(power, power);
        int i = 0;
        while (!detectWhite() && (getRuntime() - initAutoOpTime < 3)) {
            if ((++i % 10) == 0)
                driveTT(power, power);
        }
        stop_chassis();
    }

    public void goUntilWhite(double power) throws InterruptedException {
        initAutoOpTime = getRuntime();
        driveTT(power, power);
        int i = 0;
        while ((!detectWhite() || !detectWall(RANGE_WALL)) && (getRuntime() - initAutoOpTime < 3) && opModeIsActive()) {
            if ((++i % 10) == 0)
                driveTT(power, power);
        }
        stop_chassis();
    }

    public void goUntilWall(double power, double distance) throws InterruptedException {
        initAutoOpTime = getRuntime();
        driveTT(power, power);
        int i = 0;
        while (!detectWall(distance) && (getRuntime() - initAutoOpTime < 2) && opModeIsActive()) {
            if ((++i % 10) == 0)
                driveTT(power, power);
        }
        stop_chassis();
    }

    public void auto_part1(boolean is_red, boolean is_in) throws InterruptedException {

        set_pusher(PUSHER_DOWN_1);

        if (use_gyro) {
            DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                    gyro.getHeading(), leftPower, rightPower));
        }

        if (!opModeIsActive()) {  // change true to skip part1
            return;
        }
        //sleep(300);

        if (use_gyro) {
            DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                    gyro.getHeading(), leftPower, rightPower));
        }

        if (is_in) {
            StraightIn(1.0, 46);
        } else {
            StraightIn(1.0, 91);
        }
        sleep(200);

        if (is_red) {
            TurnRightD(0.35, 40, true);
        } else {
            TurnLeftD(0.35, 38, true);
        }
        StraightIn(0.75, 14);


        if (use_gyro) {
            DbgLog.msg(String.format("Gyro current heading = %d, power L/R = %.2f/%.2f",
                    gyro.getHeading(), leftPower, rightPower));
        }
        // driveTT(0.5,0.5); sleep(1);driveTT(0,0);
    }

    public void auto_part2(boolean is_red, boolean is_in, boolean is_shooting, boolean do_second_beacon, boolean is_hitting_ball) throws InterruptedException {


        //StraightIn(-0.5, 7);

        if (is_shooting) {
            if (do_second_beacon) {
                if (opModeIsActive()) {
                    goBeaconAndShooting(true, is_red);
                }
                StraightIn(1.0, 13.5); //Go forward from shooting to return to the original position
                // turn parallel to beacon
                if (is_red) {
                    float degree = 86;
                    if (use_navx) {
                        degree = 45 - (navx_device.getYaw());
                    } else if (use_ada_imu) {
                        degree = (float) (360 + ada_imu_heading() + 45);
                    } else if (use_gyro) {
                        degree = (360 - gyro.getHeading() + 45);
                    }
                    if (degree < 45){
                        degree = 86;
                    }
                    if (degree >= 180) degree = 179;
                    TurnRightD(0.35, degree, true);
                    StraightIn(1.0, 41);
                    goBeacon(true);
                } else { // blue
                    float degree = 88;
                    if (use_navx) {
                        degree = (navx_device.getYaw() + 47);
                    } else if (use_ada_imu) {
                        degree = (float) (47 - (int) ada_imu_heading());
                    } else if (use_gyro) {
                        degree = (float) (gyro.getHeading() + 47);
                    }
                    if (degree >= 180) degree = 179;
                    TurnLeftD(0.35, degree, true);
                    StraightIn(1.0, 46);
                    goBeacon(false);
                } if(is_hitting_ball){
                    StraightIn(-0.75, 10);
                    if(is_red){
                        TurnRightD(0.6, 37, true);
                    }
                    else{
                        TurnLeftD(0.6, 33, true);
                    }
                    StraightIn(-1.0, 60);
                }
            } else if (is_hitting_ball) {
                if (is_red) {
                    goBeacon(true);
                } else {
                    goBeacon(false);
                }
                StraightIn(-0.75, 25);
                goShooting(2, is_red, true);
                goBall(is_red, is_in);
            }
        } else {
            if (is_red) {
                goBeacon(true);
            } else {
                goBeacon(false);
            }
        }

        stop_tobot();
    }

    public void auto_out_shooting(boolean is_red, boolean is_ball) throws InterruptedException {

        StraightIn(-0.5, 15);
        if (is_red) {
            TurnLeftD(0.35, 28, true);
            StraightIn(-0.6, 8);
            sleep(7000);
            goShooting(2, true, false);
            TurnLeftD(0.35, 45, true);
        } else {
            TurnRightD(0.35, 37, true);
            StraightIn(-0.6, 8);
            sleep(7000);
            goShooting(2, false, false);
            TurnRightD(0.35, 38, true);
        }
        StraightIn(-0.6, 50);
        if (is_red) {
            TurnLeftD(0.35, 40, true);
            if(is_ball) {
                StraightIn(0.5, 38);
                StraightIn(-0.5,38);
            }
            else{
                StraightIn(-0.5, 17);
            }
        } else {
            TurnRightD(0.35, 40, true);
            if (is_ball) {
                StraightIn(0.5, 38);
                StraightIn(-0.5, 38);
            }
            else {
                StraightIn(-0.5, 15);
            }
        }
    }

    public void goShooting(int times, boolean is_red, boolean first_beacon) throws InterruptedException {
        if (is_red) {
            if (first_beacon) {
                //StraightIn(-0.5, 7);
                sleep(400);
                //TurnRightD(0.35,1,true);
            } else {

            }
        } else { // blue zone
            if (first_beacon) {
                //StraightIn(-0.5, 7);
                sleep(400);
                //TurnLeftD(0.35,1,true);
            } else {

            }
        }

            shooter.setPower(0.5);
            sleep(100);

            if (SH_power * 1.1 < 1.0) {
                shooter.setPower(SH_power * 1.1);
            } else {
                shooter.setPower(1.0);
            }
            sleep(1000);


            shootBallGolfGate();

        if(times == 2){
            sleep(1000);
            set_golf_gate(GOLF_GATE_OPEN);
            sleep(400);
            set_golf_gate(GOLF_GATE_CLOSED);
        }


        shooter.setPower(0);

    }

    public void goBall(boolean is_red, boolean is_in) throws InterruptedException {
        if (is_in) {
            StraightIn(-0.6, 20);
            if (is_red) {
                TurnRightD(0.35, 60, true);
                TurnLeftD(0.35, 60, true);
            } else {
                TurnLeftD(0.35, 60, true);
                TurnRightD(0.35, 60, true);
            }
            StraightIn(-0.4, 7);
        } else { // out position
            StraightIn(-0.6, 5);
            if (is_red) {
                TurnRightD(0.35, 45, true);
            } else {
                TurnLeftD(0.35, 45, true);
            }
            StraightIn(-0.6, 55);
            if (is_red) {
                TurnRightD(0.35, 60, true);
                TurnLeftD(0.35, 60, true);
            } else {
                TurnLeftD(0.35, 60, true);
                TurnRightD(0.35, 60, true);
            }
            StraightIn(-0.4, 7);
        }
    }

    public void goBeacon(boolean is_red) throws InterruptedException {
        if (!opModeIsActive()) return;

        boolean isFirstBeacon = false;
        double distanceToWall = 66.1;
        // if(use_range){
        if (false) {
            if (rangeSensor.getDistance(DistanceUnit.CM) >= 140) {
                isFirstBeacon = true;
                distanceToWall = 180.2;
            }
            goUntilWall(0.3, distanceToWall);
            StraightIn(-0.5, 2.5);
        } else if (use_ods && opModeIsActive()) {
            stopAtWhite(0.3);
            if (is_red)
                StraightIn(-0.5, 6);
            else {
                StraightIn(-0.5, 6.5);
            }
        }
        // Turn 90" toward beacon
        if (opModeIsActive()) {
            double degree;
            //sleep(200);
            if (is_red) {
                degree = 85;
                if (use_navx) {
                    degree = navx_device.getYaw() + 44.5;
                } else if (use_ada_imu) {
                    degree = (float) (44.5 - ada_imu_heading());
                } else if (use_gyro) {
                    degree = gyro.getHeading() + 44;
                }
                if (degree >= 180) degree = 179;
                TurnLeftD(0.35, (float) degree, true);
            } else { // blue
                degree = 90;
                if (use_navx) {
                    degree = (45 - navx_device.getYaw());
                } else if (use_ada_imu) {
                    degree = (float) (360 + ada_imu_heading() + 45);
                } else if (use_gyro) {
                    degree = (360 - gyro.getHeading() + 45);
                }
                if (degree >= 180) degree = 179;
                TurnRightD(0.35, (float) degree, true);
            }
            //StraightIn(-0.5, 3);
        }
        //sleep(200);
        //forwardTillUltra(10, 0.25, 3);
        blue_detected = false;
        red_detected = false;

        if (opModeIsActive()) {
            //sleep(1000);
            // Follow line until optical distance sensor detect 0.2 value to the wall (about 6cm)
            forwardTillUltra(12, 0.25, 7, is_red);
            sleep(100);

            // StraightIn(0.3, 1.0);
            //hit_left_button();
            TT_ColorPicker.Color left_co = TT_ColorPicker.Color.UNKNOWN;
            double initTime = getRuntime();
            // sense color up to 2 secs
            do {
                left_co = colorPicker.getColor(true); // get left Beacon
            }
            while ((left_co == TT_ColorPicker.Color.UNKNOWN) && (getRuntime() - initTime < 0.5) && opModeIsActive());

            if ((left_co == TT_ColorPicker.Color.UNKNOWN) && opModeIsActive()) { // Try right beacon color
                TT_ColorPicker.Color right_co = TT_ColorPicker.Color.UNKNOWN;
                do {
                    right_co = colorPicker.getColor(false);
                }
                while ((right_co == TT_ColorPicker.Color.UNKNOWN) && (getRuntime() - initTime < 1.0) && opModeIsActive());
                if (right_co == TT_ColorPicker.Color.BLUE)
                    left_co = TT_ColorPicker.Color.RED;
                else if (right_co == TT_ColorPicker.Color.RED)
                    left_co = TT_ColorPicker.Color.BLUE;
            }
            // Detect Beacon color and hit the correct side
            if (!opModeIsActive()) return;
            if (left_co == TT_ColorPicker.Color.BLUE) {
                blue_detected = true;
                if (is_red) {
                    hit_right_button();
                } else {
                    hit_left_button();
                }
            } else if (left_co == TT_ColorPicker.Color.RED) {
                red_detected = true;
                if (is_red) {
                    hit_left_button();
                } else {
                    hit_right_button();
                }
            } else { // unknown, better not do anything than giving the credit to the opponent
                // doing nothing. May print out the message for debugging
            }
        }
    }

    public void goBeaconAndShooting(boolean shoot_twice, boolean is_red) throws InterruptedException {
        if (!opModeIsActive()) return;

        boolean isFirstBeacon = false;
        double distanceToWall = 66.1;
        double shooterPW = SH_power * 1.1;
        if (shooterPW > 1.0)
            shooterPW = 1.0;
        if (use_range) {
            if (rangeSensor.getDistance(DistanceUnit.CM) >= 140) {
                isFirstBeacon = true;
                distanceToWall = 180.2;
            }
        }

        if (opModeIsActive()) {
            if (use_ods) {
                stopAtWhite(0.2);
                StraightIn(-0.5, 5.25);
            } else {
                goUntilWall(0.2, distanceToWall);
                StraightIn(-0.5, 2.5);
            }
            shooter.setPower(0.5);
            // turn toward beacon
            float degree;
            if (is_red) {
                degree = 85;
                if (use_navx) {
                    degree = navx_device.getYaw() + 46;
                } else if (use_ada_imu) {
                    degree = (float) (46 - ada_imu_heading());
                } else if (use_gyro) {
                    degree = gyro.getHeading() + 46;
                }
                if (degree >= 180) degree = 179;
                TurnLeftD(0.35, degree, true);
            } else { // blue
                degree = 90;
                if (use_navx) {
                    degree = (45 - navx_device.getYaw());
                } else if (use_ada_imu) {
                    degree = (float) (360 + ada_imu_heading() + 45);
                } else if (use_gyro) {
                    degree = (360 - gyro.getHeading() + 45);
                }
                if (degree >= 180) degree = 179;
                TurnRightD(0.35, degree, true);
            }

            shooter.setPower(shooterPW);
            //set_pusher(PUSHER_DOWN_1);
            //StraightIn(-0.5, 3);
        }
        // sleep(200);
        //forwardTillUltra(10, 0.25, 3);
        blue_detected = false;
        red_detected = false;

        if (true) {
            //sleep(1000);
            // Follow line until optical distance sensor detect 0.2 value to the wall (about 6cm)
            forwardTillUltra(12, 0.25, 7, is_red);
            sleep(100);

            // StraightIn(0.3, 1.0);
            //hit_left_button();
            TT_ColorPicker.Color left_co = TT_ColorPicker.Color.UNKNOWN;
            double initTime = getRuntime();
            // sense color up to 2 secs
            do {
                left_co = colorPicker.getColor(true);
            }
            while ((left_co == TT_ColorPicker.Color.UNKNOWN) && (getRuntime() - initTime < 0.5) && opModeIsActive());

            if ((left_co == TT_ColorPicker.Color.UNKNOWN) && opModeIsActive()) { // Try right beacon color
                TT_ColorPicker.Color right_co = TT_ColorPicker.Color.UNKNOWN;
                do {
                    right_co = colorPicker.getColor(false);
                }
                while ((right_co == TT_ColorPicker.Color.UNKNOWN) && (getRuntime() - initTime < 1.0) && opModeIsActive());
                if (right_co == TT_ColorPicker.Color.BLUE)
                    left_co = TT_ColorPicker.Color.RED;
                else if (right_co == TT_ColorPicker.Color.RED)
                    left_co = TT_ColorPicker.Color.BLUE;
            }
            // Detect Beacon color and hit the correct side
            if (!opModeIsActive())
                return;
            if (left_co == TT_ColorPicker.Color.BLUE) {
                blue_detected = true;
                if (is_red) {
                    hit_right_button();
                } else {
                    hit_left_button();
                }
            } else if (left_co == TT_ColorPicker.Color.RED) {
                red_detected = true;
                if (is_red) {
                    hit_left_button();
                } else {
                    hit_right_button();
                }
            } else { // unknown, better not do anything than giving the credit to the opponent
                // doing nothing. May print out the message for debugging
            }

        }
        if (opModeIsActive()) {
            StraightIn(-0.75, 14);
            if(is_red){
                TurnRightD(0.35, 1, true); // shoot towards center vortex
            }
            else {
                TurnRightD(0.35, 2, true);
            }

            shootAuto(shoot_twice, shooterPW);
        }
    }

    public boolean detectWhite() {
        int cur_sum_ada_colors = 0;
        double light = 0;

        if (use_adacolor) {
            cur_sum_ada_colors = coAda.alpha() + coAda.blue() + coAda.red() + coAda.green();

            if (cur_sum_ada_colors >= WHITE_ADA) {
                return true;
            }
        } else if (use_light) {
            light = lightSensor.getRawLightDetected();

            if (light >= WHITE_NXT) {
                return true;
            }
        } else if (use_ods) {
            light = odsSensor.getRawLightDetected();

            if (light >= WHITE_ODS) {
                return true;
            }
        }
        //    if (opSensor.getLightDetected() < WHITE_OP) { // to-do
        //        return false;
        //    }
        return false;
    }

    public boolean detectWall(double distance) {
        if (!use_range) {

            return false;
        }
        double range = rangeSensor.getDistance(DistanceUnit.CM);
        if (range > distance) {
            return false;
        }
        return true;
        //    if (opSensor.getLightDetected() < WHITE_OP) { // to-do
        //        return false;
        //    }

    }

    void m_warning_message(String p_exception_message) {
        if (v_warning_generated) {
            v_warning_message += ", ";
        }
        v_warning_generated = true;
        v_warning_message += p_exception_message;

    }

    public enum State {
        STATE_TELEOP,    // state to test teleop
        STATE_AUTO,        // state to test auto routines
        STATE_TUNEUP    // state to manually tune up servo positions and arm positions
    }
}
