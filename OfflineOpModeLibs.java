/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package OfflineClasses;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;

//import CoachCode.CoachFunctions.CoachDriveTest;
//import CoachCode.CoachFunctions.CoachDriveFunction;
import CoachCode.CoachOpMode.CoachBasicAuto;
import CoachCode.CoachRobot.CoachHardware;


/**
 * This is NOT an opmode.
 *
 *
 */

public class OfflineOpModeLibs extends CoachBasicAuto{

//    public CoachDriveFunction CoachDrive = new CoachDriveFunction();
//        public CoachDriveTest CoachDrive = new CoachDriveTest();

//    public Telemetry telemetry = new Telemetry();
//    public FakeTelemetry telemetry = new FakeTelemetry();


//****************************************
//DECLARE VARIABLES FROM CODE BEING TESTED
//****************************************


//****************************************
// DECLARE VARIABLES NEEDED FOR TEST CODE
//****************************************

    int counter;
    final static int size = 300;
//    int[] totalCounts = new int[size];
    int[] flCounts = new int[size];
    int[] frCounts = new int[size];
    int[] brCounts = new int[size];
    int[] blCounts = new int[size];
    int[] flIMU = new int[size];
    int[] frIMU = new int[size];
    int[] brIMU = new int[size];
    int[] blIMU = new int[size];
    int[] lsIMU = new int[size];
    double[] arrayRobotX = new double[size];
    double[] arrayRobotY= new double[size];
    double[] arrayRobotDist = new double[size];
    double[] arrayRobotAngle = new double[size];

    public double[] arrayFLBR=new double[size];
    public double[] arrayFRBL=new double[size];

    double[] arrayFieldX = new double[size];
    double[] arrayFieldY= new double[size];
    double[] arrayFieldDist = new double[size];

//    double distanceTraveledInchAlt;
//    static double[] altDistArray = new double[size];
    static double[] distanceTraveledArray= new double[size];;
    double[] timeArray= new double[size];
//    static int maxCounts = (int) Math.round(driveDistance*ROBOT_INCH_TO_MOTOR_DEG*DEGREES_TO_COUNTS);
    public double timeStep = 1000 * 30/size;//determined a fixed time step (in milliseconds) so that faster speeds will show shorter time to distance
    //timeStep was 100 in seconds to fill 30 seconds / size of array (counts to 30
    static FileReader in = null;
    static FileWriter out = null;
    static FileOutputStream fileOutStream = null;
    static DataOutputStream dataOutStream = null;

    int IMUCounter =0;

    public boolean opModeIsRunning = true;

    /* Constructor */
    public OfflineOpModeLibs(){

//        this.RobotVincent.imu.timeStep = this.timeStep;
//        this.RobotVincent.frontLeft.timeStep = this.timeStep;
//        this.RobotVincent.frontRight.timeStep = this.timeStep;
//        this.RobotVincent.backRight.timeStep = this.timeStep;
//        this.RobotVincent.backLeft.timeStep = this.timeStep;
//        this.RobotVincent.landingSlide.timeStep = this.timeStep;
//        this.RobotVincent.servoSampling.timeStep = this.timeStep;
//        this.RobotVincent.colorSensorSampling.timeStep = this.timeStep;
//
//
//        //Setting counter to capture array data is unique to offline running of code
//        counter = 1;
//        RobotVincent.imu.counter = counter;
    };


    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - EXTRACTS ARRAY DATA FROM classes
    //----------------------------------------------------------------------------------------------
    public void extractArrayData(){

//        telemetry.addData("Distance", " Command(%.2f), Current(%.2f)", commandInch, distanceTraveledInch);
//        telemetry.addData("control loop:", "Current Error(%.1f), Sum Error(%.1f), Steering Power(%.1f)", error, sumError, steerInput);
//        telemetry.update();
        flIMU = RobotVincent.imu.flArray;
        frIMU = RobotVincent.imu.frArray;
        brIMU = RobotVincent.imu.brArray;
        blIMU = RobotVincent.imu.blArray;
        lsIMU = RobotVincent.imu.lsArray;
        timeArray = RobotVincent.imu.timeArray;
        arrayRobotX = RobotVincent.imu.robotXArray;
        arrayRobotY = RobotVincent.imu.robotYArray;
        arrayRobotDist = RobotVincent.imu.robotDistArray;
        arrayRobotAngle = RobotVincent.imu.robotAngleArray;
        IMUCounter = RobotVincent.imu.counter;

        arrayFLBR = RobotVincent.imu.FLBRArray;
        arrayFRBL = RobotVincent.imu.FRBLArray;

//        robotX = RobotVincent.imu.robotX;
//        robotY = RobotVincent.imu.robotY;
//        robotDist = RobotVincent.imu.robotDist;
//        robotAngle = (double) RobotVincent.imu.fakeAngle;
        arrayFieldX = RobotVincent.imu.fieldXArray;
        arrayFieldY= RobotVincent.imu.fieldYArray;
        arrayFieldDist = RobotVincent.imu.fieldDistArray;

        Arrays.fill(flIMU,IMUCounter,(size), flIMU[IMUCounter-1]);
        Arrays.fill(frIMU,IMUCounter,(size), frIMU[IMUCounter-1]);
        Arrays.fill(brIMU,IMUCounter,(size), brIMU[IMUCounter-1]);
        Arrays.fill(blIMU,IMUCounter,(size), blIMU[IMUCounter-1]);


        Arrays.fill(arrayFLBR,IMUCounter,(size),arrayFLBR[IMUCounter-1]);
        Arrays.fill(arrayFRBL,IMUCounter,(size),arrayFRBL[IMUCounter-1]);

        Arrays.fill(arrayRobotX,IMUCounter,(size),arrayRobotX[IMUCounter-1]);
        Arrays.fill(arrayRobotY,IMUCounter,(size),arrayRobotY[IMUCounter-1]);
        Arrays.fill(arrayRobotDist,IMUCounter,(size),arrayRobotDist[IMUCounter-1]);
        Arrays.fill(arrayRobotAngle,IMUCounter,(size),arrayRobotAngle[IMUCounter-1]);

//        Arrays.fill(RX,counter,(size),RX[counter-1]);
//        Arrays.fill(RY,counter,(size),RY[counter-1]);

        Arrays.fill(arrayFieldX,IMUCounter,(size),arrayFieldX[IMUCounter-1]);
        Arrays.fill(arrayFieldY,IMUCounter,(size),arrayFieldY[IMUCounter-1]);
        Arrays.fill(arrayFieldDist,IMUCounter,(size),arrayFieldDist[IMUCounter-1]);
//        Arrays.fill(FX,counter,(size),FX[counter-1]);
//        Arrays.fill(FY,counter,(size),FY[counter-1]);
//        Arrays.fill(FDist,counter,(size),FDist[counter-1]);

        Arrays.fill(distanceTraveledArray,IMUCounter, size, distanceTraveledArray[IMUCounter-1]);
//        Arrays.fill(altDistArray,counter,(size),altDistArray[counter-1]);
        double deltaTime = (timeArray[1] - timeArray[0]);
        for(int k = IMUCounter; k < size;k++){
            timeArray[k] = timeArray[k-1] + deltaTime;

        }

    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - WRITES DATA TO FILE
    //----------------------------------------------------------------------------------------------
    public void writeToFile( FileWriter out, DataOutputStream fileData){
        int countVar = Math.max(size, IMUCounter);

        try {
            out.write(String.format("Calculated IMU Counts\r"));
            out.write(String.format("Time,FL_IMU,FR_IMU,BR_IMU,BL_IMU,LS_IMU,FLBR_cnt,FRBL_cnt, RobotY, RobotX, RobotAngle, Field_Y, Field_X\r"));

            for (int i = 0; i < countVar; i++) {
//                out.write(String.format("%d,%d,%d,%d,%d,", i,  flCounts[i], frCounts[i], brCounts[i], blCounts[i]));
                out.write(String.format("%.3f,%d,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
                        timeArray[i], flIMU[i], frIMU[i], brIMU[i], blIMU[i], lsIMU[i],
                        arrayFLBR[i],arrayFRBL[i],arrayRobotY[i], arrayRobotX[i], arrayRobotAngle[i],
                        arrayFieldY[i], arrayFieldX[i]));
                out.write(String.format("\n"));
            }

//            out.write(String.format("Robot Motion Final Values\n"));
//            out.write(String.format("Robot final rotation angle (deg.):,%.3f\n", arrayRobotAngle[size-1]));
//            out.write(String.format("Robot final X distance (in.):,%.3f \n", arrayRobotX[size-1]));
//            out.write(String.format("Robot final Y distance (in.):,%.3f \n", arrayRobotY[size-1]));
//            out.write(String.format("Robot final total distance (in.):,%.3f \n", arrayRobotDist[size-1]));

//
//            out.write(String.format("Final Distance:,%.3f\r", distanceTraveledArray[distanceTraveledArray.length - 1]));
//            out.write(String.format("Final Error:,%.3f\r", arrayRobotDist[size-1] - distanceTraveledArray[distanceTraveledArray.length - 1]));

//            out.write(String.format("Distance Traveled Array Values:\r"));
//            out.write(String.format("Time, code Dist Travel, IMU RobotY, IMU Robot X, IMU RobotDist, IMU Angle, IMU Field Y, IMU Field X, IMU Field Dist\r"));
//            for (int j = 0; j < distanceTraveledArray.length; j++) {
//                out.write(String.format("%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f\r",
//                        timeArray[j], distanceTraveledArray[j],
//                        arrayRobotY[j], arrayRobotX[j], arrayRobotDist[j], arrayRobotAngle[j],
//                        arrayFieldY[j], arrayFieldX[j], arrayFieldDist[j]));
//            }
            if (out != null) {
                out.close();
            }
        }catch(IOException e){
            e.printStackTrace();

        }

        try {
//          Write the data in binary format that can be read back in by the Java plotting programs in IntelliJ
//          Only writing out the robot X,Y, Theta as formatted for reading in

            for (int x = 0; x < countVar; x++) {
                // writes the bytes for each double in the array
                fileData.writeDouble(arrayFieldX[x]);   // robot position on field in X in inches
                fileData.writeDouble(arrayFieldY[x]);   // robot position on field in Y in inches
                fileData.writeDouble(Math.toRadians(arrayRobotAngle[x]));   // robot angle on field in radians
            }
            if(fileData != null){
                fileData.flush();
                fileData.close();
            }

        }
        catch(IOException e){
            e.printStackTrace();

        }

    }
    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - WRITES DATA TO SCREEN
    //----------------------------------------------------------------------------------------------
    public void writeToScreen(){

        System.out.println(String.format("Calculated Counts\r"));
        System.out.println(String.format("Time,FLcalc,FRcalc,BRcalc,BLcalc,FLBR_cnt, FRBL_cnt\r"));
        int countVar = Math.max(size, IMUCounter);
        for(int i=0; i<countVar; i++) {
//            System.out.print(String.format("%d,%d,%d,%d,%d,", i, flCounts[i], frCounts[i], brCounts[i], blCounts[i]));
            System.out.print(String.format("%.3f,%d,%d,%d,%d,%.1f,%.1f",timeArray[i], flIMU[i], frIMU[i], brIMU[i], blIMU[i],
                    arrayFLBR[i],arrayFRBL[i]));
            System.out.println(String.format(" "));
        }
        System.out.println(String.format("-----------------------"));
        System.out.println("Robot Motion Final Values");
        System.out.println(String.format("%.3f Robot final rotation angle (deg.)",arrayRobotAngle[size-1]));
        System.out.println(String.format("%.3f Robot final X distance (in.)",arrayRobotX[size-1]));
        System.out.println(String.format("%.3f Robot final Y distance (in.)",arrayRobotY[size-1]));
        System.out.println(String.format("%.3f Robot final total distance (in.)",arrayRobotDist[size-1]));
        System.out.println(String.format("-----------------------"));

//        System.out.println(String.format("Final Distance: %.3f",distanceTraveledArray[distanceTraveledArray.length-1]));
//        System.out.println(String.format("Final Error: %.3f", arrayRobotDist[size-1] - distanceTraveledArray[distanceTraveledArray.length-1]));
//        System.out.println(String.format("-----------------------"));
        System.out.println(String.format(" "));

        System.out.println("Distance Traveled Array Values");
        System.out.println("Time,Distance Travel, FieldX, FieldY, RobotDist, Angle");

        for(int j =0; j<distanceTraveledArray.length; j++){
            System.out.println(String.format("%.3f,%.3f,%.3f,%.3f,%.3f,%.1f\r",timeArray[j],distanceTraveledArray[j],arrayFieldX[j],arrayFieldY[j],arrayRobotDist[j],arrayRobotAngle[j]));

        }
    }
    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - UPDATES FAKE IMU
    //----------------------------------------------------------------------------------------------
    @Override
    public void updateIMU(){
        RobotVincent.imu.flCnt = RobotVincent.frontLeft.getCurrentPosition();
        RobotVincent.imu.frCnt = RobotVincent.frontRight.getCurrentPosition();
        RobotVincent.imu.brCnt = RobotVincent.backRight.getCurrentPosition();
        RobotVincent.imu.blCnt = RobotVincent.backLeft.getCurrentPosition();
        RobotVincent.imu.lsCnt = RobotVincent.landingSlide.getCurrentPosition();
        RobotVincent.imu.serPos = RobotVincent.servoSampling.getPosition();

        try {
//                distanceTraveledArray[counter] = distanceTraveledInch;
               counter += 1;
//               RobotVincent.imu.counter = counter;
//                RobotVincent.imu.updateCounter(counter);
                if (counter >= size) {
                    int a = 1 / 0;
                }
            } catch (ArithmeticException e) {
                System.out.println(String.format("Exceeded %d counter steps", size));
//                distanceTraveledInch = driveDistance;
            }


    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD prepOpMode is used to initialize offline items
    //----------------------------------------------------------------------------------------------
   public void prepOpMode() {
       //This is the offline version of init that has additional items
       //Needs to be run before runOpMode or inherited init
       //Added code to try to use library version of hardware
       RobotVincent.TestMode = true;
       RobotVincent.init(hardwareMap);

       RobotVincent.imu.timeStep = timeStep;
       RobotVincent.frontLeft.timeStep = timeStep;
       RobotVincent.frontRight.timeStep = timeStep;
       RobotVincent.backRight.timeStep = timeStep;
       RobotVincent.backLeft.timeStep = timeStep;
       RobotVincent.landingSlide.timeStep = timeStep;
       RobotVincent.servoSampling.timeStep = timeStep;
       RobotVincent.colorSensorSampling.timeStep = timeStep;

       //Setting counter to capture array data is unique to offline running of code
       counter = 1;
       RobotVincent.imu.counter = counter;
       RobotVincent.robotNumber =1;
       if(RobotVincent.robotNumber == 1) {

           //field angle orientation is + = CCW , while robot frame is + = CW
           RobotVincent.imu.fieldX = 62;//initial x position on field in inches
           RobotVincent.imu.fieldY = -24;//initial y position on field in inches
           RobotVincent.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST
           RobotVincent.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST
       }


       if(RobotVincent.robotNumber == 2) {
           //field angle orientation is + = CCW , while robot frame is + = CW
           RobotVincent.imu.fieldX = 62;//initial x position on field in inches
           RobotVincent.imu.fieldY = 24;//initial y position on field in inches
           RobotVincent.imu.priorAngle = 180;//initial robot angle orientation on field in degrees from EAST
           RobotVincent.imu.fakeAngle = 180;//initial robot angle orientation on field in degrees from EAST

       }
       //Initialize starting position on field, field cent eris assumed (0,0), 0 field angle is pointing EAST
       RobotVincent.imu.fieldXArray[0] = RobotVincent.imu.fieldX; //initial x position on field in inches
       RobotVincent.imu.fieldYArray[0] = RobotVincent.imu.fieldY; //initial y position on field in inches
       RobotVincent.imu.robotAngleArray[0] = RobotVincent.imu.priorAngle; //initial robot angle orientation on field in degrees from EAST


   }

    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // INSERT ACTUAL CODE TO BE TESTED IN METHODS
    //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD runAutonomous is populated with actual program code in runOpMode
    //----------------------------------------------------------------------------------------------
    @Override
    public void initOpMode() {

        // IF statement needs to be in main init function, but only utilized on robot
        if(!testMode){
            RobotVincent.init(hardwareMap);
        }
        //COLLAPSED ITEMS BELOW HAVE MOTOR CONFIGURATIONS - DO NOT CHANGE
        //Set Arm motor configuration - Do Not Edit
        RobotVincent.landingSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RobotVincent.landingSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotVincent.landingSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotVincent.landingSlide.setPower(0);
        RobotVincent.landingSlide.setTargetPosition(0);

        //Motor configuration, recommend Not Changing - Set all motors to forward direction, positive = clockwise when viewed from shaft side
        RobotVincent.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        RobotVincent.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        RobotVincent.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        RobotVincent.backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        RobotVincent.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RobotVincent.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RobotVincent.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        RobotVincent.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Reset all motor encoders
        RobotVincent.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotVincent.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotVincent.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RobotVincent.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set all motors to position mode (assumes that all motors have encoders on them)
        RobotVincent.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotVincent.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotVincent.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RobotVincent.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        RobotVincent.frontLeft.setTargetPosition(0);
        RobotVincent.frontRight.setTargetPosition(0);
        RobotVincent.backLeft.setTargetPosition(0);
        RobotVincent.backRight.setTargetPosition(0);

        RobotVincent.frontLeft.setPower(0);
        RobotVincent.frontRight.setPower(0);
        RobotVincent.backLeft.setPower(0);
        RobotVincent.backRight.setPower(0);

        RobotVincent.servoSampling.setPosition(0);


    }
    @Override
    public void runOpMode(){
        initOpMode(); // run initialization from init in this OpMode
//        super.initOpMode(); //run initialization inherited from CoachBasicAuton

        CoachDrive.angleUnWrap(this);
        //Indicate initialization complete and provide telemetry
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                RobotVincent.frontLeft.getPower(), RobotVincent.frontRight.getPower(),
                RobotVincent.backLeft.getPower(), RobotVincent.backRight.getPower());
        telemetry.addData("Motors Pos", "FL (%d), FR (%d), BL (%d), BR (%d)",
                RobotVincent.frontLeft.getCurrentPosition(), RobotVincent.frontRight.getCurrentPosition(),
                RobotVincent.backLeft.getCurrentPosition(), RobotVincent.backRight.getCurrentPosition());
        telemetry.addData("Target Positions", "Forward (%d), Right (%d), Rotate (%d)",
                forwardPosition,rightPosition,clockwisePosition);
        telemetry.addData("heading", "%.2f",RobotVincent.robotHeading);
//        telemetry.addData("OpMode Active","%d",testOpMode);
        telemetry.update();//Update telemetry to update display

        runtime.reset();

        RobotVincent.angles = RobotVincent.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//This line calls the angles from the IMU
        double rollOffset = RobotVincent.angles.secondAngle;

        startTime = runtime.time();
//            while(!isStarted()){

            RobotVincent.angles = RobotVincent.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Roll angle: ","( %.2f )", RobotVincent.angles.secondAngle - rollOffset);
            //telemetry.addData("Pitch angle: ","( %.2f )", angles.thirdAngle);
            telemetry.addData("heading", "%.2f",RobotVincent.robotHeading);
//            telemetry.addData("OpMode Active","%d",testOpMode);
            telemetry.addData(">", "Press Play to start"); //display this on screen
            telemetry.update();//Update telemetry to update display
//            }

        //Complete initialization and telemetry
        //**** PUSHING START BUTTON WILL RUN AUTONOMOUS CODE ******
//            waitForStart();

        runtime.reset(); //reset counter to start with OpMode

//        RobotVincent.angles = RobotVincent.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);//This line calls the angles from the IMU
        startTime = runtime.time();

//        RobotVincent.offset = RobotVincent.angles.firstAngle; //Determine initial angle offset 
//        RobotVincent.priorAngle = RobotVincent.offset; //set prior angle for unwrap to be initial angle 
//        RobotVincent.robotHeading = RobotVincent.angles.firstAngle - RobotVincent.offset; //robotHeading to be 0 degrees to start 

        while (runtime.time() - startTime < 0.1) {
            //wait for initial angle data capture prior to running code
        }

//        targetColorRange[0] = 15;
//        targetColorRange[1] = 30;




        CoachDrive.setPower(0,this);//simplify using method


//        RobotVincent.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RobotVincent.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RobotVincent.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        RobotVincent.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set motor power limits - for "Run_to_Position" this sets the maximum speed
//        RobotVincent.frontLeft.setPower(paramFile.DRIVE_POWER_LIMIT);
//        RobotVincent.frontRight.setPower(paramFile.DRIVE_POWER_LIMIT);
//        RobotVincent.backLeft.setPower(paramFile.DRIVE_POWER_LIMIT);
//        RobotVincent.backRight.setPower(paramFile.DRIVE_POWER_LIMIT);

        CoachDrive.angleUnWrap(this);
        telemetry.addData("OpMode Running", "%s",testOpMode);
        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                RobotVincent.frontLeft.getPower(), RobotVincent.frontRight.getPower(),
                RobotVincent.backLeft.getPower(), RobotVincent.backRight.getPower());
        telemetry.addData("Motors Pos", "FL (%d), FR (%d), BL (%d), BR (%d)",
                RobotVincent.frontLeft.getCurrentPosition(), RobotVincent.frontRight.getCurrentPosition(),
                RobotVincent.backLeft.getCurrentPosition(), RobotVincent.backRight.getCurrentPosition());
        telemetry.addData("Target Positions", "Forward (%d), Right (%d), Rotate (%d)",
                forwardPosition,rightPosition,clockwisePosition);
        telemetry.addData("heading", "%.2f",RobotVincent.robotHeading);



        if(RobotVincent.robotNumber == 1) {
            //This is Skystone auto robot
//            rightPosition = (int) Math.round(2  * paramFile.adjustedRight *  paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
//            CoachDrive.driveGeneral(rightPosition, paramFile.DRIVE_POWER_LIMIT, 1,"Move 2 inches Right", this);
//            telemetry.addLine("**** Completed Initialize Motors ****");
//            telemetry.update();
//
//            clockwisePosition = (int) Math.round((5.85 * paramFile.adjustedRotate) *
//                    paramFile.ROBOT_DEG_TO_WHEEL_INCH * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
////        CoachDrive.driveRotate(clockwisePosition, paramFile.ROTATE_POWER_LIMIT, "Rotating IMU value deg. counterclockwise", this);
//            CoachDrive.driveGeneral(clockwisePosition, paramFile.ROTATE_POWER_LIMIT, 2,"Rotating To West", this);
//            telemetry.addLine("**** Completed Rotate to Face West ****");
//            telemetry.update();
//
//            //Offline code is having issue where 2 of 4 motors move farther on initial step and robot rotates
//            rightPosition = (int) Math.round(-2  * paramFile.adjustedRight *  paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
//            CoachDrive.driveGeneral(rightPosition, paramFile.DRIVE_POWER_LIMIT, 1,"Move 2 inches Left", this);
//            telemetry.addLine("**** Completed Return to Start ****");
//            telemetry.update();

            forwardPosition = (int) Math.round(24 * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(forwardPosition, paramFile.DRIVE_POWER_LIMIT, 0, "Go 24 inches FWD", this);
            telemetry.addLine("**** Completed Move Forward 24 inches ****");
            telemetry.update();

            clockwisePosition = (int) Math.round((-90 * paramFile.adjustedRotate) * paramFile.ROBOT_DEG_TO_WHEEL_INCH * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(clockwisePosition, paramFile.ROTATE_POWER_LIMIT, 2, "Rotating 90 deg. Counterclockwise", this);
            telemetry.addLine("**** Completed Rotate to Face Towards Audience ****");
            telemetry.update();

            forwardPosition = (int) Math.round(24 * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(forwardPosition, paramFile.DRIVE_POWER_LIMIT, 0, "Go 24 inches forward", this);
            telemetry.addLine("**** Completed Skystone Search ****");
            telemetry.update();


            rightPosition = (int) Math.round(6 * paramFile.adjustedRight * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(rightPosition, paramFile.DRIVE_POWER_LIMIT, 1, "Move 6 inches Right", this);
            telemetry.addLine("**** Completed Position for SkyStone Collection ****");
            telemetry.update();

            for (int j = 0; j < 15; j++) {
                CoachDrive.angleUnWrap(this);
                telemetry.addLine("**** Performing Skystone Collection  - non driving ****");
                telemetry.update();
            }
            telemetry.addLine("**** Completed Skystone Collection ****");
            telemetry.update();

            rightPosition = (int) Math.round(-7 * paramFile.adjustedRight * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS; // needs testing
            CoachDrive.driveGeneral(rightPosition, paramFile.DRIVE_POWER_LIMIT, 1, "Go 7 inches Left", this);
            telemetry.addLine("**** Completed Move Left 7 inches ****");
            telemetry.update();

            clockwisePosition = (int) Math.round((180 * paramFile.adjustedRotate) *paramFile.ROBOT_DEG_TO_WHEEL_INCH * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(clockwisePosition, paramFile.ROTATE_POWER_LIMIT, 2,"Rotating 180 deg. clockwise", this);
            telemetry.addLine("**** Completed Rotate 180 deg CW ****");
            telemetry.update();

            forwardPosition = (int) Math.round(72 * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(forwardPosition, paramFile.DRIVE_POWER_LIMIT, 0,"Go 72 inches forward", this);
            telemetry.addLine("**** Completed Move Forward 72 inches ****");
            telemetry.update();

            for (int j = 0; j < 5; j++) {
                CoachDrive.angleUnWrap(this);
                telemetry.addLine("**** Performing Skystone Drop  - non driving ****");
                telemetry.update();
            }
            telemetry.addLine("**** Completed Skystone Drop ****");
            telemetry.update();

            forwardPosition = (int) Math.round(-24 * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(forwardPosition, paramFile.DRIVE_POWER_LIMIT, 0,"Go 24 inches backward", this);
            telemetry.addLine("**** Completed Move Backward 24 inches ****");
            telemetry.update();

        }
        if(RobotVincent.robotNumber == 2) {
//            rightPosition = (int) Math.round(2  * paramFile.adjustedRight *  paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
//            CoachDrive.driveGeneral(rightPosition, paramFile.DRIVE_POWER_LIMIT, 1,"Move 2 inches Right", this);
//            telemetry.addLine("**** Completed Initialize Motors ****");
//            telemetry.update();
//
//            clockwisePosition = (int) Math.round((5.85 * paramFile.adjustedRotate) *
//                    paramFile.ROBOT_DEG_TO_WHEEL_INCH * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
////        CoachDrive.driveRotate(clockwisePosition, paramFile.ROTATE_POWER_LIMIT, "Rotating IMU value deg. counterclockwise", this);
//            CoachDrive.driveGeneral(clockwisePosition, paramFile.ROTATE_POWER_LIMIT, 2,"Rotating To West", this);
//            telemetry.addLine("**** Completed Rotate to Face West ****");
//            telemetry.update();
//
//            //Offline code is having issue where 2 of 4 motors move farther on initial step and robot rotates
//            rightPosition = (int) Math.round(-2  * paramFile.adjustedRight *  paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
//            CoachDrive.driveGeneral(rightPosition, paramFile.DRIVE_POWER_LIMIT, 1,"Move 2 inches Left", this);
//            telemetry.addLine("**** Completed Return to Start ****");
//            telemetry.update();

            forwardPosition = (int) Math.round(24 * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(forwardPosition, paramFile.DRIVE_POWER_LIMIT/2, 0, "Go 24 inches FWD", this);
            telemetry.addLine("**** Completed Move Forward 24 inches ****");
            telemetry.update();

            CoachDrive.angleUnWrap(this);

            clockwisePosition = (int) Math.round((90 * paramFile.adjustedRotate) * paramFile.ROBOT_DEG_TO_WHEEL_INCH * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(clockwisePosition, paramFile.ROTATE_POWER_LIMIT, 2, "Rotating 90 deg. clockwise", this);
            telemetry.addLine("**** Completed Rotate to Face Away from Audience ****");
            telemetry.update();

            forwardPosition = (int) Math.round(24 * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(forwardPosition, paramFile.DRIVE_POWER_LIMIT/2, 0, "Go 24 inches forward", this);
            telemetry.addLine("**** Completed Foundation Search ****");
            telemetry.update();


            rightPosition = (int) Math.round(-6 * paramFile.adjustedRight * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(rightPosition, paramFile.DRIVE_POWER_LIMIT/2, 1, "Move 6 inches Left", this);
            telemetry.addLine("**** Completed Position for SkyStone Collection ****");
            telemetry.update();

            for (int j = 0; j < 15; j++) {
                CoachDrive.angleUnWrap(this);
                telemetry.addLine("**** Performing Foundation Capture - non driving ****");
                telemetry.update();
            }
            telemetry.addLine("**** Completed Foundation Capture ****");
            telemetry.update();

            rightPosition = (int) Math.round(24 * paramFile.adjustedRight * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS; // needs testing
            CoachDrive.driveGeneral(rightPosition, paramFile.DRIVE_POWER_LIMIT/2, 1, "Go 30 inches Right", this);
            telemetry.addLine("**** Completed Move Left 30 inches ****");
            telemetry.update();
            for (int j = 0; j < 5; j++) {
                CoachDrive.angleUnWrap(this);
                telemetry.addLine("**** Performing Foundation Drop  - non driving ****");
                telemetry.update();
            }

            telemetry.addLine("**** Completed Deliver Foundation ****");
            telemetry.update();

            forwardPosition = (int) Math.round(-48 * paramFile.ROBOT_INCH_TO_MOTOR_DEG) * paramFile.DEGREES_TO_COUNTS;
            CoachDrive.driveGeneral(forwardPosition, paramFile.DRIVE_POWER_LIMIT/2, 0,"Go 48 inches backward", this);
            telemetry.addLine("**** Completed Move Backward 48 inches ****");
            telemetry.update();
        }




        } //MAIN OpMode PROGRAM END



    //##############################################################################################
    // END ACTUAL CODE TO BE TESTED AS METHODS
    //##############################################################################################

    //Run Calculations like Autonomous OpMode
    public static void main(String []args)throws IOException {

// Code to setup the main program that runs offline, none of this is robot code
        OfflineOpModeLibs OffLibs = new OfflineOpModeLibs();
        OffLibs.testMode = true;//Declare Test Mode
        OffLibs.testOpMode = true;
        // Prepare robot class for offline operation, must be run prior to copied runOpMode or init
        // Sets initial position and counters and initial array variables
        OffLibs.prepOpMode();


        StringBuilder fileName = new StringBuilder();
        fileName.append("RMO_");
        fileName.append(String.format("%s.csv","RuckusAutonomous01"));
        out = new FileWriter(fileName.toString());
        //============= SET ROBOT 1 or 2 to WRITE DATA ==============
//        fileOutStream = new FileOutputStream("C:/Users/Spiessbach/Documents/GitHub/OfflineViz/RobotOnField.dat");// Path to directory for IntelliJ code
//        fileOutStream = new FileOutputStream("C:/Users/Spiessbach/Documents/GitHub/OfflineCode/Robot1OnField.dat");// Path to directory for IntelliJ code
        fileOutStream = new FileOutputStream("C:/Users/Spiessbach/Documents/GitHub/OfflineCode/"+String.format("Robot%dOnField.dat",OffLibs.RobotVincent.robotNumber));// Path to directory for IntelliJ code

        dataOutStream = new DataOutputStream(fileOutStream);
//calls code input by programmer into runAutonomous method that comes from main runOpMode
        OffLibs.runOpMode();

// Lines below are to capture the array data and output
        OffLibs.extractArrayData();
        OffLibs.writeToFile(out, dataOutStream);
//        OffLibs.writeToScreen();
        fileOutStream.close();

    }


}