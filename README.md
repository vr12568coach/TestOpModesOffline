# TestOpModesOffline
Test bed code to run OpModes in Android Studio on local machine and output data data files
This code emulates hardware interfaces used on the robot, contains field configuration information,
and OfflineOpModes that run team code offline as a simulation

# 1/22/2021 UPDATED DIRECTIONS FOR INSTALLATION & USE
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
12. Update OfflineOpModeLIbs.jav for your path to the RobotVisualization

    Find the line of code "OffLibs.location = computer.KARL;//For Karl on Mac" update this to CALEB OR WILL
    Find the block of code
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

