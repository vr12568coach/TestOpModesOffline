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

package TestOpModesOffline;


import com.qualcomm.robotcore.hardware.DcMotorSimple;

import CoachCode.CoachFunctions.CoachDriveFunction;
import CoachCode.CoachFunctions.CoachParam;
import CoachCode.CoachOpMode.CoachBasicAuto;
import CoachCode.CoachOpMode.CoachDrivingAuto;
import TestOpModesOffline.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;

import CoachCode.CoachOpMode.CoachBasicOpMode;


/**
 * This is NOT an opmode.
 *
 *
 */

public class OfflineOpModeLibs extends CoachBasicAuto {


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

        try {
//
                if (IMUCounter >= size) {
                    int a = 1 / 0;
                }
            } catch (ArithmeticException e) {
                System.out.println(String.format("Exceeded %d counter steps", size));
                System.out.println(String.format("IMU Counter = %d", IMUCounter));
//                distanceTraveledInch = driveDistance;
            }


    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD prepOpMode is used to initialize offline items - instead of initOpMode
    //----------------------------------------------------------------------------------------------
   public void prepOpMode() {

       //This is the offline version of init that has additional items
       //Needs to be run before runOpMode or inherited init
       //Added code to try to use library version of hardware
       RobotVincent.TestMode = true;
       // Map all of hardware to program from robot using HardwareVincent script
       //Will refer all defined configuration items - motors, servos, sensors- to this defined RobotVincent class
//        createRobot();

       RobotVincent.init(hardwareMap);

       //***************** MODIFIED TO LOAD FROM PC ********************
       telemetry.addData("Loading from","%s",fileNameEdited);
       telemetry.addData("loadFile ","%s",loadFile);
       telemetry.update();

       //Change method to loadfromPC
       constants.phm = constants.loadFromPC(fileNameEdited, constants.phm,this);
       if(!loadFile) {
//           HashMap hasn't been created, need to create file
           telemetry.addLine("HashMap File did not Exist");
           telemetry.addLine(String.format("loadFile =  %s",loadFile));
           telemetry.update();
           telemetry.addLine(String.format("Defining HashMap and Writing file to %s", fileNameEdited));
           constants.defineParams();
           //Change to savetoFile
           constants.saveToFile(fileNameEdited, constants.phm);
           telemetry.addLine(String.format("loadFile =  %s",loadFile));
           telemetry.update();

       }
       else {
           telemetry.addLine("HashMap File Successfully Loaded");
           telemetry.addLine(String.format("loadFile =  %s",loadFile));
           telemetry.update();
       }
       telemetry.addLine("HashMap Load Completed");
       telemetry.addLine("-------------------------------------------");

       telemetry.update();

       //*************** END MODIFIED FOR LOAD ***************************************
       //Motor configuration, recommend Not Changing - Set all motors to forward direction, positive = clockwise when viewed from shaft side
       RobotVincent.frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
       RobotVincent.frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
       RobotVincent.backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
       RobotVincent.backRight.setDirection(DcMotorSimple.Direction.FORWARD);

       RobotVincent.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       RobotVincent.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       RobotVincent.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       RobotVincent.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       RobotVincent.frontLeft.setTargetPosition(0);
       RobotVincent.frontRight.setTargetPosition(0);
       RobotVincent.backLeft.setTargetPosition(0);
       RobotVincent.backRight.setTargetPosition(0);
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


       RobotVincent.frontLeft.setPower(0);
       RobotVincent.frontRight.setPower(0);
       RobotVincent.backRight.setPower(0);
       RobotVincent.backLeft.setPower(0);

       //***********************************************************
       //Code that needs to be Kept in init to initialize functions
       //***********************************************************

       RobotVincent.imu.timeStep = timeStep;
       RobotVincent.frontLeft.timeStep = timeStep;
       RobotVincent.frontRight.timeStep = timeStep;
       RobotVincent.backRight.timeStep = timeStep;
       RobotVincent.backLeft.timeStep = timeStep;

       //Setting counter to capture array data is unique to offline running of code
       counter = 1;
       RobotVincent.imu.counter = counter;
       RobotVincent.robotNumber = 2;
       if(RobotVincent.robotNumber == 1) {
           constants.phm.get("DRIVE_POWER_LIMIT").setValue(1.0);
           constants.phm.get("squareDistance").setValue(48);
//           RobotVincent.frontLeft.motorTol=1.2;
//           RobotVincent.backLeft.motorTol=1.2;
           //field angle orientation is + = CCW , while robot frame is + = CW
           RobotVincent.imu.fieldX = 60;//initial x position on field in inches
           RobotVincent.imu.fieldY = -12;//initial y position on field in inches
           RobotVincent.imu.priorAngle = -90;//initial robot angle orientation on field in degrees from EAST
           RobotVincent.imu.fakeAngle = -90;//initial robot angle orientation on field in degrees from EAST
       }


       if(RobotVincent.robotNumber == 2) {

           constants.phm.get("DRIVE_POWER_LIMIT").setValue(0.5);
           constants.phm.get("squareDistance").setValue(32);
//           RobotVincent.frontLeft.motorTol=1.2;
           RobotVincent.backLeft.motorTol=1.2;

           //field angle orientation is + = CCW , while robot frame is + = CW
           RobotVincent.imu.fieldX = 24;//initial x position on field in inches
           RobotVincent.imu.fieldY = 24;//initial y position on field in inches
           RobotVincent.imu.priorAngle = 90;//initial robot angle orientation on field in degrees from EAST
           RobotVincent.imu.fakeAngle = 90;//initial robot angle orientation on field in degrees from EAST
           telemetry.addData("Robot Number ", "%d",RobotVincent.robotNumber);
           telemetry.addData("DRIVE_POWER_LIMIT ", "%.2f",constants.phm.get("DRIVE_POWER_LIMIT").value);
           telemetry.addData("squareDistance ", "%.2f",constants.phm.get("squareDistance").value);


           telemetry.update();
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


    }
    @Override
    public void runOpMode(){
        initOpMode();
        //Indicate initialization complete and provide telemetry
        telemetry.addData("Status: ", "Initialized");
        telemetry.addData("Drive Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)",
                RobotVincent.frontLeft.getPower(), RobotVincent.frontRight.getPower(),
                RobotVincent.backLeft.getPower(), RobotVincent.backRight.getPower());
        telemetry.addData("Target Positions", "Forward (%d), Right (%d), Rotate (%d)",
                forwardPosition,rightPosition,clockwisePosition);
        telemetry.update();//Update telemetry to update display

        runtime.reset();


        //Complete initialization and telemetry
        //**** PUSHING START BUTTON WILL RUN AUTONOMOUS CODE ******
//        waitForStart();

        runtime.reset(); //reset counter to start with OpMode

        driveSquare(constants.phm.get("squareDistance").value);
        constants.phm.get("DRIVE_POWER_LIMIT").setValue(constants.phm.get("DRIVE_POWER_LIMIT").value/2);

        driveSquare(constants.phm.get("squareDistance").value/2);

    } //MAIN OpMode PROGRAM END


    //##############################################################################################
    // END ACTUAL CODE TO BE TESTED AS METHODS
    //##############################################################################################

    //Run Calculations like Autonomous OpMode
    public static void main(String []args)throws IOException {

// Code to setup the main program that runs offline, none of this is robot code
        OfflineOpModeLibs OffLibs = new OfflineOpModeLibs();
        OffLibs.testMode = true;//Declare Test Mode
        OffLibs.opModeIsRunning = true;
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
        fileOutStream = new FileOutputStream("C:/Users/Spiessbach/Documents/FTC/IntelliJ Projects/RobotVisualization/"+String.format("Robot%dOnField.dat",OffLibs.RobotVincent.robotNumber));// Path to directory for IntelliJ code

        dataOutStream = new DataOutputStream(fileOutStream);
//calls code input by programmer into runAutonomous method that comes from main runOpMode
        OffLibs.runOpMode();
// Lines below are to capture the array data and output
        OffLibs.extractArrayData();
        OffLibs.writeToFile(out, dataOutStream);
//        OffLibs.writeToScreen();
        fileOutStream.close();
        OffLibs.telemetry.addData("Total Number of Time Steps", "%d",OffLibs.IMUCounter);
        OffLibs.telemetry.update();
    }


}