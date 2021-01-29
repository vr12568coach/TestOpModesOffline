# TestOpModesOffline
Test bed code to run OpModes in Android Studio on local machine and output data data files
This code emulates hardware interfaces used on the robot, contains field configuration information,
and OfflineOpModes that run team code offline as a simulation

# 1/29/2021 UPDATED DIRECTIONS FOR INSTALLATION & USE
Prior to cloning this project the steps below must be completed:
1. Create a copy of the FTC project that the team is using to develop robot code
    For example: if you have an Android Studio project "UltimateGoalFTCApp", then copy that directory
    and rename the copy to something like "UltimateGoalOffline"
2. Create a folder for the offline project in the new directory name "OfflineCode"
    Folder must be located in the TeamCode\src\main\java\ directory
    FOlder must have the name "OfflineCode" in order for all the package names to be the same

Clone this repository from github.com
1. Click the "Code" button at the upper right of site
2. Select Open With GitHub desktop
3. Select the folder created above as the location to clone the project to
    Make sure that the after selecting that folder that the entire path is correct and the final folder is
    named "OfflineCode"
4. Clone the repository
5. Create a directory for the robot code that will be used offline
   within /TeamCode/src/main/java/ create the fodler "UltimateGoal_RobotTeam" to match the package name for the robot code
6. Select the UltimateGoal_1 branch in GitHub desktop and update the repository

IMPORTANT - follow the instructions below in order to use this code
1. The Android Studio code that is being used on the robot must be copied over into this project
    DO NOT CLONE THE ROBOT PROJECT HERE - follow directions below
    There are changes needed to run the robot code in the offline simulation
2. on github.com find the vf12568coach/RobotVisualization repository
3. Clone that repository to another code directory
    The directory should be outside the "UltimateGoalOffline" project but it can be in any file location
    Note: You will need the file location for updating the OfflineOpModeLibs file for the path to write files
    Note: Select the V3_in_work branch in GitHub desktop and update the repository

4. Download IntelliJ IDEA or other code development package for Java that is not targeted for Android phones
    You need this package to open and run the RobotVisualization Code
5. Open the "RobotVisualization" project in IntelliJ
6. Open File_IO.java
7. Modify the strings for the following paths
      static String mainPathString = "/Users/karl/LocalDocuments/FTC/UltimateGoal/Code/UltimateGoalOfflineNew/TeamCode/src/main/java/UltimateGoal_RobotTeam";
        Update this to your path to the UltimateGoalOffline project - this is where you will be writing files for creating offline code

      static File directoryPath = new File("/Users/karl/LocalDocuments/FTC/UltimateGoal/Code/UltimateGoalFTCApp/TeamCode/src/main/java/UltimateGoal_RobotTeam");
        Update this to your path to the robot code project where "UltimateGoal_RobotTeam" is the folder that has the code you run on the phones
        This is where you will be copying files from
8. Run File_IO
    Click on the "start" green triangle next to public static void main(String args[]) throws IOException
    This will run the code and copy the on phone files to offline files
9. Check the screen output in IntelliJ for errors copying files or creating directories
10. Do not edit the Offline repository "UltimateGoal_RobotTeam" files
    When you need to make changes make them in the on robot code that will go on the phones
    Then run File_IO (no changes should be needed) to copy the files to the Offline version
11. If you need to make modification to the robot code to allow it to run Offline
    add lines to the checkOfflineExceptions() method
    This method reads lines from the robto code and replaces them to be offline compatible
# [1/29/21] Updated so there are 2 choices for Offline Code
 12. More Tedious Option: OfflineOpModeLIbs.java where you have to copy in the OpMode content
#   AWESOME OPtion OfflineOpModeRunFile.java - just instantiate the OpMOde you want to run

 13. Update enum for your path to the RobotVisualization - common to both files

    A) for OfflineOpModeLibs: Find the line of code "OffLibs.location = computer.KARL;//For Karl on Mac"
    update this to CALEB OR WILL
    B) for OfflineOpModeRunFile: Find the line of code "OffRunFiles.location = computer.KARL;//For Karl on Mac"
        update this to CALEB OR WILL
14. Common to both types: Find the block of code
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
                   fileLocation = "/";
                   break;
           }
       Replace the file path by your name with the path to the RobotVisualization project on your machine

15. Running OfflineOpModeRunFile
    Find Code inputs and loops
    a) find line 544: int numRings = 0;//Set ring number here to be used for all 4 robots
    This is the int variable that is changed to set the field configuration

    b) find line 548 that reads for(int h = 1; h<3;h++) {// After creating "Do Nothing" OpModes change to h < 5
    This is the start of a loop that can run 4 robots worth of OpModes
    In order to run 4 robots you need 4 OpModes - 2 blue and 2 red.  Of course all robots can run the same OpMode but that just makes a mess
    Get familiar with this block of code and what needs to changed to implement an OpMode

     Line 550: CompleteAutonomousBlueExterior inputOpMode = new CompleteAutonomousBlueExterior();

     This changes Robot #1 code to run from the instantiated opMode
     For a new OpMode just replace the "CompleteAutonomousBlueExterior" with the OpMode you want

     c) What's needed in an OpMode to run it here?
     Find line 584  - this the actual execution of the code
                 OffRunFiles.robotNumber = h;
                 OffRunFiles.prepOpMode(numRings);//Prep field with rings
                 OffRunFiles.autoOpMode.constructRobot();//Construct robot using Phone OpMode Code
                 OffRunFiles.initOfflineIMU();//Pass robot location into Offline HW
                 OffRunFiles.autoOpMode.initialize();//Initialize robot  using Phone OpMode Code
                 OffRunFiles.autoOpMode.runCode();//Run OpMode Code
     Your OpMode must have these methods
        constructRobot()
        initialize()
        runCode()

     and the BasicAuto class must have the latest updateIMU() method for tracking robot and field locations

     As long as you have these methods these lines of OfflineOpModeRunFile should never need to change

     d) Once the OpModes are instantiated and your path for hte output files have been updated

     Run the File_IO.java from RobotVisualization project to make sure you have the latest Offline OCde
     Run OfflineOpModeRUnFile main() method with coverage to execute the code and create the output files
     Update the h < "integer" value in the loop to run 2 or up to 4 robots

 16. After running the main method Run RobotOnFieldVizMain main() method from the RobotVisualization project
 This will create the javaFx window to create the robot images

 17. Change the ring configuration variable and repeat!


