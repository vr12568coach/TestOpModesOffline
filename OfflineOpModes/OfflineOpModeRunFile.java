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

package OfflineCode.OfflineOpModes;


import java.io.DataOutputStream;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.Arrays;


import OfflineCode.Field.FieldConfiguration;
import UltimateGoal_RobotTeam.HarwareConfig.HardwareRobotMulti;
import UltimateGoal_RobotTeam.OpModes.Autonomous.BasicAuto;
import UltimateGoal_RobotTeam.OpModes.Autonomous.BlueExPowerShotAuto;
import UltimateGoal_RobotTeam.OpModes.Autonomous.BlueInPowerShotAuto;
import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingBlueInt;
import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingRedExt;
import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingRedInt;
import UltimateGoal_RobotTeam.OpModes.Autonomous.MainBlueExAuto;
import UltimateGoal_RobotTeam.OpModes.Autonomous.MainBlueInAuto;

import UltimateGoal_RobotTeam.OpModes.Autonomous.CompleteAutonomousBlueExterior;
import UltimateGoal_RobotTeam.OpModes.Autonomous.CompleteAutonomousBlueInterior;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingBlueExt;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingBlueInt;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingRedExt;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.DoNothingRedInt;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.RedExteriorWobbleHighGoal;
//import UltimateGoal_RobotTeam.OpModes.Autonomous.RedInteriorWobbleHighGoal;
import UltimateGoal_RobotTeam.OpModes.Autonomous.RedExteriorWobbleHighGoal;
import UltimateGoal_RobotTeam.OpModes.Autonomous.RedInteriorWobbleHighGoal;
import UltimateGoal_RobotTeam.Utilities.PursuitLines;

//import com.qualcomm.robotcore.hardware.HardwareMap;

//************************************************************************************************
//** THIS FILE INSTANTIATES OpModes FROM THE OpModeCSV.csv FILE AND RUNS THEM FOR VISUALIZATION **
//************************************************************************************************

public class OfflineOpModeRunFile extends BasicAuto {

//****************************************
// DECLARE VARIABLES NEEDED FOR TEST CODE
//****************************************
    //Define the size of the arrays and number of points that are allowed to run
    static int runPoints =300;//'size' is defined in BasicAuto for overall Array size as 3000 to be much larger than runPoints needs to be
    //Define the points per second that will be passed to the Robot Visualization
    static int pointsPerSecond = 10;
    //Define based on calculation the end time in seconds for the simulation
    static double tEnd = (double)runPoints/(double)pointsPerSecond;
    private int robotNumber = 1;

//Instantiate the Autonomous OpMode you wish to Run - can be multiple for each of 4 robots
// Below being used as the surrogate robot so multiple autonomous OpModes can be contained in a single run
  public BasicAuto autoOpMode = new BasicAuto();// Needed for Offline Code


    int counter;

    public double timeStep = 135;//determined a fixed time step (in milliseconds) so that faster speeds will show shorter time to distance
    /* Moved time step to BasicOpMode */


    static FileReader in = null;
    static FileWriter out = null;
    static FileOutputStream fileOutStream = null;
    static DataOutputStream dataOutStream = null;
    static DataOutputStream dos = null;
    static FileOutputStream fos = null;
    public String SelectedOpMode = null;

    static String fileLocation;
    static computer location;

    public boolean opModeIsRunning = true;


    //Constructor
    public OfflineOpModeRunFile(BasicAuto inputOpMode){
        autoOpMode = inputOpMode;
    };
    public OfflineOpModeRunFile(){

    };


    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - EXTRACTS ARRAY DATA FROM classes
    //----------------------------------------------------------------------------------------------
    public void extractArrayData(){


        IMUCounter = autoOpMode.robotUG.driveTrain.imu.counter;

        telemetry.addData("IMU Counters","Main = %d autoOpMode = %d, and size of RobotPoints = %d",IMUCounter,autoOpMode.IMUCounter,autoOpMode.fc.RobotPoints.size());
        telemetry.addData("List Size","Ring = %d, Wobble Goal = %d, Pursuit = %d, Nav = %d",
                autoOpMode.fc.BlueRingPoints.size(),autoOpMode.fc.BlueWobble1Points.size(),autoOpMode.fc.PursuitPoints.size(),autoOpMode.fc.NavPoints.size());
        telemetry.addData("Array Size"," collectorArray = %d, WGA = %d,",
                autoOpMode.collectorArray.length,autoOpMode.wgaAngleArray.length);
        telemetry.update();

        Arrays.fill(autoOpMode.collectorArray,IMUCounter,(runPoints-1), autoOpMode.collectorArray[IMUCounter-1]);
        Arrays.fill(autoOpMode.conveyorArray,IMUCounter,(runPoints-1), autoOpMode.conveyorArray[IMUCounter-1]);
        Arrays.fill(autoOpMode.shooterArray,IMUCounter,(runPoints-1), autoOpMode.shooterArray[IMUCounter-1]);
        Arrays.fill(autoOpMode.wgaAngleArray,IMUCounter,(runPoints-1), autoOpMode.wgaAngleArray[IMUCounter-1]);

        for(int k = IMUCounter; k < runPoints;k++){// updated to only add points
            autoOpMode.fc.RobotPoints.add(autoOpMode.fc.RobotPoints.get(k-1));

            autoOpMode.fc.BlueRingPoints.add(autoOpMode.fc.BlueRingPoints.get(k-1));
            autoOpMode.fc.RedRingPoints.add(autoOpMode.fc.RedRingPoints.get(k-1));

            autoOpMode.fc.RedWobble1Points.add(autoOpMode.fc.RedWobble1Points.get(k-1));
            autoOpMode.fc.BlueWobble1Points.add(autoOpMode.fc.BlueWobble1Points.get(k-1));

            autoOpMode.fc.BlueWobble2Points.add(autoOpMode.fc.BlueWobble2Points.get(k-1));
            autoOpMode.fc.RedWobble2Points.add(autoOpMode.fc.RedWobble2Points.get(k-1));
            autoOpMode.fc.PursuitPoints.add(autoOpMode.fc.PursuitPoints.get(k-1));
            autoOpMode.fc.NavPoints.add(autoOpMode.fc.NavPoints.get(k-1));

        }

    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE CODE ONLY - WRITES DATA TO FILE
    //----------------------------------------------------------------------------------------------
    public void writeExtrasToFile(FileOutputStream fos){
        int countVar = Math.max(runPoints, IMUCounter);


        try {

            OutputStreamWriter osw = new OutputStreamWriter(fos);

//          Write the data in text format that can be read back in by the Java visualization programs in IntelliJ
//          Only writing out the power or position as formatted for reading in, Used for accessory parts that are always attached to robot
            osw.write("Collector(On/Off)\tConveyor(On/Off)\tShooter(On/Off)\tWGA_Angle(rad.)\n"); // Header Row

            for (int j = 0; j < runPoints; j++) {
                // writes the data as text for each value in the array
                osw.write(Integer.toString(autoOpMode.collectorArray[j])+"\t");   // collector Power - use to determine if ON
                osw.write(Integer.toString(autoOpMode.conveyorArray[j])+"\t");   // conveyor Power - use to determine if ON
                osw.write(Integer.toString(autoOpMode.shooterArray[j])+"\t");   // shooter Power - use to determine if ON
                osw.write(Double.toString(autoOpMode.wgaAngleArray[j])+"\n");   // Wobble Goal Arm angle / motion
            }
            if(osw != null){
                osw.flush();
                osw.close();
            }

        }
        catch(IOException e){
            e.printStackTrace();
            System.out.println(String.format("Error occurred","%S",e));
        }


    }
    public void writeMetaData(FileOutputStream fos){

        try {

            OutputStreamWriter osw = new OutputStreamWriter(fos);

//          Write the data in text format that can be read back in by the Java visualization programs in IntelliJ
//          Only writing out certain variables that were used to set the number of points in the run or number of points per second
            osw.write(Integer.toString(runPoints)+"\t");   // total number of points
            osw.write(Integer.toString(pointsPerSecond)+"\n");   // points per second

            if(osw != null){
                osw.flush();
                osw.close();
            }

        }
        catch(IOException e){
            e.printStackTrace();
            System.out.println(String.format("Error occurred","%S",e));
        }


    }
    public void writePath(FileOutputStream fos, ArrayList<PursuitLines> lines, int numPoints){

        try {

            OutputStreamWriter osw = new OutputStreamWriter(fos);

//          Write the data in text format that can be read back in by the Java visualization programs in IntelliJ
//          Only writing out the FieldLocation X,Y, Theta as formatted for reading in, Used for foundations,stones, robot, and gripper
            osw.write("X1 (in.)"+"\t"+"Y1 (in.)"+"\t"+"X2 (in.)"+"\t"+"Y2 (in.)"+"\n"); // Header Row

            for (int j = 0; j < numPoints; j++) {
                // writes the bytes for each double in the array
                osw.write(Double.toString(lines.get(j).x1)+"\t"); // Path X1 position on field in inches
                osw.write(Double.toString(lines.get(j).y1)+"\t");   // Path Y1 position on field in inches
                osw.write(Double.toString(lines.get(j).x2)+"\t"); // Path X2 position on field in inches
                osw.write(Double.toString(lines.get(j).y2)+"\n");   // FPath Y2 position on field in inches
            }
            if(osw != null){
                osw.flush();
                osw.close();
            }

        }
        catch(IOException e){
            e.printStackTrace();
            System.out.println(String.format("Error occurred","%S",e));
        }

    }

    //----------------------------------------------------------------------------------------------
    // TEST MODE METHOD prepOpMode is used to initialize offline items - instead of initOpMode
    //----------------------------------------------------------------------------------------------
   public void prepOpMode(int ringNumber) {

   //************* BELOW IS TEST CODE ********************************


       autoOpMode.testModeActive = true;// set for each OpMode


       autoOpMode.ringSelect = ringNumber;// Options are 0, 1, 4 rings on the field
       autoOpMode.fc = new FieldConfiguration(ringSelect);//KS added 12/20 to set stone position

       autoOpMode.haveBlueRing = false;
       autoOpMode.haveRedRing = false;
       autoOpMode.haveBlueWobble1 = false;
       autoOpMode.haveBlueWobble2 = false;
       autoOpMode.haveRedWobble1 = false;
       autoOpMode.haveRedWobble2 = false;

       writeBR = false;
       writeRR = false;
       writeBW1 = false;
       writeBW2 = false;
       writeRW1 = false;
       writeRW2 = false;

       switch(location) {
           case KARL:
               fileLocation = "/Users/karl/LocalDocuments/FTC/IntelliJ/RobotVisualization/";
               break;
           case PC:
               fileLocation = "C:/Users/Spiessbach/Documents/FTC/IntelliJ Projects/RobotVisualization/";
               break;
           case MAC:
               fileLocation = "/Users/caleb/Documents/FTC/IntelliJ/RobotVisualization/";
               break;
           case WILL:
               fileLocation = "C:/Users/young/Desktop/Android Studio/RobotVisualization/";
               break;
       }

       //************* ABOVE IS TEST CODE ********************************

//       MOVED timeStep to constructors on HW items for test mode

       autoOpMode.fc.RedRingPoints.clear();
       autoOpMode.fc.BlueRingPoints.clear();
       autoOpMode.fc.BlueWobble1Points.clear();
       autoOpMode.fc.RedWobble1Points.clear();
       autoOpMode.fc.BlueWobble2Points.clear();
       autoOpMode.fc.RedWobble2Points.clear();

       collectorArray= new int[size];;
       conveyorArray= new int[size];
       shooterArray= new int[size];
       wgaAngleArray= new double[size];


   }

   public void initOfflineIMU(){
       autoOpMode.robotUG.driveTrain.imu.RobotPoints.clear();
       //Setting counter to capture array data is unique to offline running of code
       counter = 1;//Needed for Offline imu since is has compare to previous step
       autoOpMode.robotUG.driveTrain.imu.counter =  counter;//Needed robot constructed to set counter
        //Set IMU positions in Offline HW
       autoOpMode.robotUG.driveTrain.imu.robotOnField.x = autoOpMode.robotUG.driveTrain.robotFieldLocation.x;//initial x position on field in inches
       autoOpMode.robotUG.driveTrain.imu.robotOnField.y = autoOpMode.robotUG.driveTrain.robotFieldLocation.y;//initial y position on field in inches
       autoOpMode.robotUG.driveTrain.imu.robotOnField.theta = autoOpMode.robotUG.driveTrain.robotFieldLocation.theta;//initial robot angle orientation on field in degrees from EAST CCW +
       autoOpMode.robotUG.driveTrain.imu.priorAngle = autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST CCW +
       autoOpMode.robotUG.driveTrain.imu.fakeAngle = (float) autoOpMode.robotUG.driveTrain.imu.robotOnField.theta;//initial robot angle orientation on field in degrees from EAST CCW +
       autoOpMode.robotUG.driveTrain.robotHeading = -autoOpMode.robotUG.driveTrain.imu.fakeAngle;//initial robot angle orientation on field in degrees from EAST, CW +

   }

   public enum computer{PC,KARL,MAC,WILL};

    //Run Calculations like Autonomous OpMode
    public static void main(String []args)throws IOException {

        //Instantiate a static class to run the code with
        OfflineOpModeRunFile OffRunFiles = new OfflineOpModeRunFile();

        OffRunFiles.location = computer.KARL;//For Karl on HP
//  UPDATE SELECTION and PATH for your machine
//        OffRunFiles.location = computer.MAC;//For Caleb
//        OffRunFiles.location = computer.WILL;//For William



        // ********** SET RUN CONDITION & SELECT THE OPMODE ************************

        int numRings = 1;//Set ring number here to be used for all 4 robots

        runPoints = 350;//Define the number of points to capture
        pointsPerSecond = 10; //Define the points per second that will be passed to the Robot Visualization
        tEnd = (double)runPoints/(double)pointsPerSecond;//Define based on calculation the end time in seconds for the simulation
        // Set the initial conditions for EACH ROBOT

        for(int h = 1; h<3;h++) {// After creating "Do Nothing" OpModes change to h < 5
            if(h == 1){// Define robot #1
//                CompleteAutonomousBlueExterior inputOpMode = new CompleteAutonomousBlueExterior();
//                BlueExPowerShotAuto inputOpMode = new BlueExPowerShotAuto();
                MainBlueExAuto inputOpMode = new MainBlueExAuto();
                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);

                OffRunFiles.autoOpMode.writeBR=true;
                OffRunFiles.autoOpMode.writeBW1=true;
            }
            else if (h == 2){// Define robot #2
//                CompleteAutonomousBlueInterior inputOpMode = new CompleteAutonomousBlueInterior();
//                BlueInPowerShotAuto inputOpMode = new BlueInPowerShotAuto();
                DoNothingBlueInt inputOpMode = new DoNothingBlueInt();

                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);

//                OffRunFiles.autoOpMode.writeBR=true;
                OffRunFiles.autoOpMode.writeBW2=true;
            }
            else if (h == 3){// Define robot #3
//                RedInteriorWobbleHighGoal inputOpMode = new RedInteriorWobbleHighGoal();
//                DoNothingRedInt inputOpMode = new DoNothingRedInt();

//                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);

                OffRunFiles.autoOpMode.writeRR=true;
                OffRunFiles.autoOpMode.writeRW1=true;
            }
            else if (h == 4){// Define robot #4
//                RedExteriorWobbleHighGoal inputOpMode = new RedExteriorWobbleHighGoal();
//                DoNothingRedExt inputOpMode = new DoNothingRedExt();

//                OffRunFiles = new OfflineOpModeRunFile(inputOpMode);

//                OffRunFiles.autoOpMode.writeRR=true;
                OffRunFiles.autoOpMode.writeRW2=true;
            }
        // Run the code

            OffRunFiles.robotNumber = h;
            OffRunFiles.prepOpMode(numRings);//Prep field with rings
            OffRunFiles.autoOpMode.constructRobot();//Construct robot using Phone OpMode Code
            OffRunFiles.initOfflineIMU();//Pass robot location into Offline HW
            OffRunFiles.autoOpMode.initialize();//Initialize robot  using Phone OpMode Code
            OffRunFiles.autoOpMode.runCode();//Run OpMode Code

// Lines below are to capture the array data and output
            OffRunFiles.extractArrayData();
//            int countVar = Math.max(runPoints, OffRunFiles.IMUCounter);

            fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dOnField.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
            OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RobotPoints, runPoints);

            fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dAccessories.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
            OffRunFiles.writeExtrasToFile(fos);

            fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dPath.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
            OffRunFiles.writePath(fos, OffRunFiles.autoOpMode.lines, OffRunFiles.autoOpMode.lines.size());

            fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dPursuit.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
            OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.PursuitPoints, runPoints);
            fos = new FileOutputStream(OffRunFiles.fileLocation + String.format("Robot%dNav.txt", OffRunFiles.robotNumber));// Path to directory for IntelliJ code
            OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.NavPoints, runPoints);

            if (OffRunFiles.autoOpMode.writeRR) {
                fos = new FileOutputStream(fileLocation + "RedRing.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RedRingPoints, runPoints);
                fos = new FileOutputStream(fileLocation + "RingSet.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeRingType(fos);

            }
            if (OffRunFiles.autoOpMode.writeBR) {
                fos = new FileOutputStream(fileLocation + "BlueRing.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.BlueRingPoints, runPoints);
                OffRunFiles.telemetry.addData("Sizes"," points: %d, array: %d",runPoints,OffRunFiles.size);
                OffRunFiles.telemetry.update();
                fos = new FileOutputStream(fileLocation + "RingSet.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeRingType(fos);
            }
            if (OffRunFiles.autoOpMode.writeRW1) {
                fos = new FileOutputStream(fileLocation + "RedWobble1.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RedWobble1Points, runPoints);
            }
            if (OffRunFiles.autoOpMode.writeRW2){
                fos = new FileOutputStream(fileLocation + "RedWobble2.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.RedWobble2Points, runPoints);
            }
            if (OffRunFiles.autoOpMode.writeBW1 ) {
                fos = new FileOutputStream(fileLocation + "BlueWobble1.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.BlueWobble1Points, runPoints);
            }
            if ( OffRunFiles.autoOpMode.writeBW2){
                fos = new FileOutputStream(fileLocation + "BlueWobble2.txt");// Path to directory for IntelliJ code
                OffRunFiles.fc.writeFieldAsText(fos, OffRunFiles.autoOpMode.fc.BlueWobble2Points, runPoints);
            }

            OffRunFiles.telemetry.addData("Total Number of Time Steps", "%d", OffRunFiles.IMUCounter);
            OffRunFiles.telemetry.addData("Completed", "Robot: %d, side: %d, 1= BLUE, -1 = RED", h, OffRunFiles.autoOpMode.sideColor);
            OffRunFiles.telemetry.addLine("===================================");
            OffRunFiles.telemetry.addLine(" ");
            OffRunFiles.telemetry.update();

        }
        //Write out the basic timing MetaData to the screen and file
        OffRunFiles.telemetry.addLine(" ");
        OffRunFiles.telemetry.addData("Meta Data","Points: %d, End Time: %.2f, Points/Second: %d",runPoints,tEnd,pointsPerSecond);
        OffRunFiles.telemetry.addData("File Location"," %s",OffRunFiles.fileLocation);
        OffRunFiles.telemetry.addLine("===================================");
        OffRunFiles.telemetry.addLine(" ");
        OffRunFiles.telemetry.update();
        fos = new FileOutputStream(OffRunFiles.fileLocation + "RunMetaData.txt");// Path to directory for IntelliJ code and filename
        OffRunFiles.writeMetaData(fos);
    }

}