package TestOpModesOffline;

import android.content.Context;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.BufferedReader;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.HashMap;

import Skystone_14999.OpModes.Autonomous.BasicAuto;
import TestOpModesOffline.OpModeParam;
import Skystone_14999.OpModes.BasicOpMode;

public class OpModeParamFunctions {

    public HashMap<String, OpModeParam> omp = new HashMap();
    public OpModeParamFunctions(){
        //Empty constructor

    }
    public void defineParameters() {
        //Create the OpMpdes and place in the hashMap

        omp.put("RedStoneIn", new OpModeParam("Red", "Front", "In", "Moved", -180, 0,"N"));
        omp.put("RedStoneOut", new OpModeParam("Red", "Front", "Out", "Moved", -180, 0,"N"));
        omp.put("RedFoundIn", new OpModeParam("Red", "Back", "In", "None", -180, 1,"N"));
        omp.put("RedFoundOut", new OpModeParam("Red", "Back", "Out", "None", -180, 1,"N"));

        omp.put("BlueStoneIn", new OpModeParam("Blue", "Front", "In", "Moved", 0, 0,"N"));
        omp.put("BlueStoneOut", new OpModeParam("Blue", "Front", "Out", "Moved", 0, 0,"N"));
        omp.put("BlueFoundIn", new OpModeParam("Blue", "Back", "In", "None", 0, 1,"N"));
        omp.put("BlueFoundOut", new OpModeParam("Blue", "Back", "Out", "None", 0, 1,"N"));

    }// Define initial values for HashMap parameters

    public boolean loadFromCSV(String fileName, BasicOpMode om) {
        boolean loadFile = false;
        boolean readHeader = false;
        try {
            BufferedReader csvReader = new BufferedReader(new FileReader(fileName));
            String s = null;
            while ((s = csvReader.readLine()) != null) {
                String[] data = s.split("\t");
                loadFile = true;
                if(readHeader) {
                    omp.put(data[0], new OpModeParam(data[1], data[2], data[3], data[4], Double.parseDouble(data[5]), Integer.parseInt(data[6]), data[7]));
                    om.telemetry.addData("OpModeName", "%s", data[0]);
                    om.telemetry.addData("Team Color", "%s", omp.get(data[0]).teamColor);
                    om.telemetry.addData("Start Side", "%s", omp.get(data[0]).startSide);
                    om.telemetry.addData("Bridge Side", "%s", omp.get(data[0]).bridgeSide);
                    om.telemetry.addData("SkyStone Drop", "%s", omp.get(data[0]).SkyStoneDrop);
                    om.telemetry.addData("Start Direction", "%.1f", omp.get(data[0]).startDirection);
                    om.telemetry.addData("Case Select", "%d", omp.get(data[0]).caseSelect);
                    om.telemetry.addData("Selected OpMode", "%s", omp.get(data[0]).selected);
                    om.telemetry.addLine("==================================");
                    om.telemetry.update();
                }
                readHeader = true; //skip initial row because it is the header
            }

            csvReader.close();
        }catch(Exception e) {
            loadFile = false;
            om.telemetry.addData("Exception","%s", e.toString());
            om.telemetry.update();
        }
        return loadFile;
    }
    public void saveToCSV(String filename, BasicOpMode om) {

        try {
            FileOutputStream fos = new FileOutputStream(filename);
            OutputStreamWriter osw = new OutputStreamWriter(fos);
            // WRITE THE HEADER ROW

            osw.write("OpMode Name\tTeam Color\tStart Side\tBridge Side\tSkyStone Drop\tStart Direction\tCase Select\tSelected OpMode?\n");
//                om.telemetry.addLine(" ");
//                om.telemetry.addLine("OpMode Name\tTeam Color\tStart Side\tBridge Side\tSkyStone Drop\tStart Direction\tCase Select\tSelected OpMode?\n");

            // WRITE THE ROWS FROM THE HASHMAP
            for(String s : omp.keySet()){
                om.telemetry.addLine(" ");
                osw.write(s + "\t");
                om.telemetry.addData("OpMode Name", "%s", s);
                osw.write(omp.get(s).teamColor + "\t");
                om.telemetry.addData("Team Color", "%s", omp.get(s).teamColor);
                osw.write(omp.get(s).startSide + "\t");
                om.telemetry.addData("Start Side", "%s", omp.get(s).startSide);
                osw.write(omp.get(s).bridgeSide + "\t");
                om.telemetry.addData("Bridge Side", "%s", omp.get(s).bridgeSide);
                osw.write(omp.get(s).SkyStoneDrop + "\t");
                om.telemetry.addData("SkyStone Drop", "%s", omp.get(s).SkyStoneDrop);
                osw.write(omp.get(s).startDirection + "\t");
                om.telemetry.addData("Start Direction", "%.1f", omp.get(s).startDirection);
                osw.write(omp.get(s).caseSelect + "\t");
                om.telemetry.addData("Case Select", "%d", omp.get(s).caseSelect);
                osw.write(omp.get(s).selected + "\n");
                om.telemetry.addData("Selected OpMode", "%s", omp.get(s).selected );

                om.telemetry.addLine("______________________________________________");
                om.telemetry.update();

//
            }
            osw.close();
        }catch(Exception e){
//            Log.e("Exception", e.toString());
            System.out.println("There was an Error Writing");
        }
    }

    public void writeToPhone(String fileName, BasicOpMode om) {
        Context c = om.hardwareMap.appContext;

        try {

            OutputStreamWriter osw = new OutputStreamWriter(c.openFileOutput(fileName, c.MODE_PRIVATE));

            for(String s : omp.keySet()) {

                osw.write(s + "\n");
                om.telemetry.addData("OpMode Name", "%s", s);
                osw.write(omp.get(s).teamColor + "\n");
                om.telemetry.addData("Team Color", "%s", omp.get(s).teamColor);
                osw.write(omp.get(s).startSide + "\n");
                om.telemetry.addData("Start Side", "%s", omp.get(s).startSide);
                osw.write(omp.get(s).bridgeSide + "\n");
                om.telemetry.addData("Bridge Side", "%s", omp.get(s).bridgeSide);
                osw.write(omp.get(s).SkyStoneDrop + "\n");
                om.telemetry.addData("SkyStone Drop", "%s", omp.get(s).SkyStoneDrop);
                osw.write(omp.get(s).startDirection + "\n");
                om.telemetry.addData("Start Direction", "%.1f", omp.get(s).startDirection);
                osw.write(omp.get(s).caseSelect + "\n");
                om.telemetry.addData("Case Select", "%d", omp.get(s).caseSelect);
                osw.write(omp.get(s).selected + "\n");
                om.telemetry.addData("Selected OpMode", "%s", omp.get(s).selected );
                om.telemetry.update();

            }

            osw.close();
        }
        catch(Exception e) {

            Log.e("Exception", e.toString());

            om.telemetry.addData("Exception","%s", e.toString());
            om.telemetry.update();
        }
    }
    public boolean readFromPhone(String fileName, BasicOpMode om) {
        Context c = om.hardwareMap.appContext;
        boolean loadFile = false;

        try {
            InputStream is = c.openFileInput(fileName);
            InputStreamReader isr = new InputStreamReader(is);
            BufferedReader br = new BufferedReader(isr);

            String s;
            while((s = br.readLine())!= null) {
            //s reads the HashMap key which is the OpMode name or short description
                loadFile = true;

//        public String teamColor =  "Red";       // options Red/Blue
//        public String startSide = "Front";      // options Front/Back
//        public String bridgeSide = "In";        // options In/Out
//        public String SkyStoneDrop = "NotMoved";   // options NotMoved/Moved
//        public double startDirection = 0;       // options 0/180 degrees
//        public int caseSelect = 0;              // options 0,1,2,3,4... max number of cases
                String tc = br.readLine();
                String ss = br.readLine();
                String bs = br.readLine();
                String ssd = br.readLine();
                double sd = Double.parseDouble(br.readLine());
                int cs = Integer.parseInt(br.readLine());
                String sel = br.readLine();
                omp.put(s, new OpModeParam(tc, ss, bs, ssd, sd, cs, sel));

                om.telemetry.addData("OpMode Name", "%s", s);
                om.telemetry.addData("Team Color", "%s", tc);
                om.telemetry.addData("Start Side", "%s", ss);
                om.telemetry.addData("Bridge Side", "%s", bs);
                om.telemetry.addData("SkyStone Drop", "%s", ssd);
                om.telemetry.addData("Start Direction", "%.1f", sd);
                om.telemetry.addData("Case Select", "%d", cs);
                om.telemetry.addData("Selected OpMode", "%s", sel);
                om.telemetry.addLine("/////////////////////////////");

                om.idle();
            }

            is.close();
        }
        catch(Exception e) {

            Log.e("Exception", e.toString());

            loadFile = false;

            om.telemetry.addData("Exception","%s", e.toString());
            om.telemetry.update();
        }
        return loadFile;
    }

    public boolean loadFromPC(String filename, BasicOpMode om){
        boolean loadFile = false;
        try{
            FileReader in = new FileReader(filename);
            BufferedReader br = new BufferedReader(in);
            String s;
            om.telemetry.addData("Reading from"," %s", filename);

            while((s = br.readLine()) != null){
                loadFile = true;
                //s reads the HashMap key which is the OpMode name or short description

//        public String teamColor =  "Red";       // options Red/Blue
//        public String startSide = "Front";      // options Front/Back
//        public String bridgeSide = "In";        // options In/Out
//        public String SkyStoneDrop = "NotMoved";   // options NotMoved/Moved
//        public double startDirection = 0;       // options 0/180 degrees
//        public int caseSelect = 0;              // options 0,1,2,3,4... max number of cases
                String tc = br.readLine();
                String ss = br.readLine();
                String bs = br.readLine();
                String ssd = br.readLine();
                double sd = Double.parseDouble(br.readLine());
                int cs = Integer.parseInt(br.readLine());
                String sel = br.readLine();


                if(omp.get(s) != null){
                    omp.put(s, new OpModeParam(tc, ss, bs, ssd, sd, cs, sel));
                    om.telemetry.addData("Updating OpMode","Key= %s",s);

                }
                else if(omp.get(s) == null){
                    omp.put(s, new OpModeParam(tc, ss, bs, ssd, sd, cs, sel));
                    om.telemetry.addData("Adding OpMode","Key= %s",s);

                }

                om.telemetry.addData("Team Color", "%s", tc);
                om.telemetry.addData("Start Side", "%s", ss);
                om.telemetry.addData("Bridge Side", "%s", bs);
                om.telemetry.addData("SkyStone Drop", "%s", ssd);
                om.telemetry.addData("Start Direction", "%.1f", sd);
                om.telemetry.addData("Case Select", "%d", cs);
                om.telemetry.addData("Selected OpMode", "%s", sel);
                om.telemetry.addLine("______________________________________________");
//                om.pressXtoContinue();
            }
            om.telemetry.update();
            in.close();

        }catch(Exception e){
//            Log.e("Exception", e.toString());
            loadFile = false;
            om.telemetry.addData("File Does Not Exist","%s"," ");
            om.telemetry.addData("Setting Flag", "%s",loadFile);
            om.telemetry.update();
        }
        return loadFile;
    }
    public void saveToFile(String filename, BasicOpMode om){

        try{
            FileOutputStream fos = new FileOutputStream(filename);
            OutputStreamWriter osw = new OutputStreamWriter(fos);
            for(String s : omp.keySet()){
                om.telemetry.addLine(" ");
                osw.write(s + "\n");
                om.telemetry.addData("OpMode Name", "%s", s);
                osw.write(omp.get(s).teamColor + "\n");
                om.telemetry.addData("Team Color", "%s", omp.get(s).teamColor);
                osw.write(omp.get(s).startSide + "\n");
                om.telemetry.addData("Start Side", "%s", omp.get(s).startSide);
                osw.write(omp.get(s).bridgeSide + "\n");
                om.telemetry.addData("Bridge Side", "%s", omp.get(s).bridgeSide);
                osw.write(omp.get(s).SkyStoneDrop + "\n");
                om.telemetry.addData("SkyStone Drop", "%s", omp.get(s).SkyStoneDrop);
                osw.write(omp.get(s).startDirection + "\n");
                om.telemetry.addData("Start Direction", "%.1f", omp.get(s).startDirection);
                osw.write(omp.get(s).caseSelect + "\n");
                om.telemetry.addData("Case Select", "%d", omp.get(s).caseSelect);
                osw.write(omp.get(s).selected + "\n");
                om.telemetry.addData("Selected OpMode", "%s", omp.get(s).selected );

                om.telemetry.addLine("______________________________________________");
                om.telemetry.update();

//
            }
            osw.close();
        }catch(Exception e){
//            Log.e("Exception", e.toString());
            System.out.println("There was an Error Writing");
        }
    }

    public void editHashMap(BasicOpMode om) {

        for(String s : omp.keySet()) {

            while(!(om.gamepad1.x || om.gamepad1.b) && om.opModeIsActive()) {
                // X to EDIT || B to SKIP
                om.telemetry.addData("OpMode Name", "%s", s);
//                om.telemetry.addData("Team Color", "%s", omp.get(s).teamColor);
//                om.telemetry.addData("Start Side", "%s", omp.get(s).startSide);
//                om.telemetry.addData("Bridge Side", "%s", omp.get(s).bridgeSide);
//                om.telemetry.addData("SkyStone Drop", "%s", omp.get(s).SkyStoneDrop);
//                om.telemetry.addData("Start Direction", "%.1f", omp.get(s).startDirection);
//                om.telemetry.addData("Case Select", "%d", omp.get(s).caseSelect);
                om.telemetry.addData("Selected OpMode", "%s", omp.get(s).selected);
                om.telemetry.addLine("X to EDIT || B to SKIP");
                om.telemetry.update();
            }
            if(om.gamepad1.x) {

                while(!om.gamepad1.y && om.opModeIsActive()) {

                    om.telemetry.addData("OpMode Name", "%s", s);
                    om.telemetry.addData("Team Color", "%s", omp.get(s).teamColor);
                    om.telemetry.addData("Start Side", "%s", omp.get(s).startSide);
                    om.telemetry.addData("Bridge Side", "%s", omp.get(s).bridgeSide);
                    om.telemetry.addData("SkyStone Drop", "%s", omp.get(s).SkyStoneDrop);
                    om.telemetry.addData("Start Direction", "%.1f", omp.get(s).startDirection);
                    om.telemetry.addData("Case Select", "%d", omp.get(s).caseSelect);
                    om.telemetry.addData("Selected OpMode", "%s", omp.get(s).selected);
                    om.telemetry.addLine("X to EDIT || B to SKIP");
                    om.telemetry.update();
                    om.telemetry.addLine("Right Bumper to set SELECTED to 'Y', Left Bumper to set to 'N'");
                    om.telemetry.addLine("Press Y to accept value");
                    om.telemetry.update();

                    if(om.gamepad1.right_bumper) {

                        omp.get(s).setSelect("Y");
                        om.sleep(300);
                    }
                    if(om.gamepad1.left_bumper) {

                        omp.get(s).setSelect("N");
                        om.sleep(300);
                    }
                }
            }
            if(om.gamepad1.b) {

                om.telemetry.addData("Skipped","%s", s);
                om.telemetry.update();
                om.sleep(500);
            }

        }
    }

    public void checkoutFiles(String loadCSVFileName, BasicOpMode om)throws IOException {
        om.telemetry.addData("Status", "Starting Code");
        om.telemetry.update();


        om.telemetry.addData("Reading Hash Map from File", "%s", loadCSVFileName);
        om.telemetry.update();


        if (!om.ompf.loadFromCSV(loadCSVFileName, om)) {
            //HashMap hasn't been created, need to create file
            om.telemetry.addLine("File Did Not Exist, Creating Hash Map and saving to File");
            om.telemetry.update();

            om.ompf.defineParameters();
            om.ompf.saveToCSV(loadCSVFileName, om);
        }

    }

    public void writeOpMode(String filePath, String filename, OpModeParam oMP) {
        double sC = 0.0;
        double fPC = 0.0;
        double iO = 0.0;
        boolean stoneSide =false;
        if(filename.contains("Stone")){
            stoneSide =true;
        }


        //Set the parameters in the OpMode being written based on the OpModeParam passed intot he method
        if(oMP.teamColor.equals("Red")){sC = -1.0;}
        else{ sC = 1.0;}

        if(oMP.SkyStoneDrop.equals("Moved")){fPC = 0.0;}
        else{ fPC = 26.0;}

        if(oMP.bridgeSide.equals("In")){iO = 0.0;}
        else{ iO = 24.0;}

        if(oMP.startSide.equals("Front")){stoneSide =true;}
        else{ stoneSide = false;}

        try {
            FileOutputStream fos = new FileOutputStream(filePath+"/"+filename);
            OutputStreamWriter osw = new OutputStreamWriter(fos);
            //Write the basic package and import info
            osw.write("package Skystone_14999.OpModes.Autonomous.AutoGenerated;"+"\n\n");
            osw.write("import com.qualcomm.robotcore.eventloop.opmode.Autonomous;"+"\n");
            osw.write("import org.firstinspires.ftc.robotcore.external.ClassFactory;"+"\n");
//            osw.write("import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;"+"\n");
//            osw.write("import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;"+"\n");
//            osw.write("import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;"+"\n");
            osw.write("import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;"+"\n");
            osw.write("import Skystone_14999.OpModes.Autonomous.BasicAuto;"+"\n\n");


            //Create the OpMode name from the filename
            String opModeName = filename.replace(".java","");

            //Write the opMode Name and Group to display on the phone
            osw.write(String.format("@Autonomous(name=\"%s\", group=\"Autonomous\")\n\n",opModeName));

            //Write the class name declaration
            osw.write(String.format(" public class %s extends BasicAuto {\n\n",opModeName));

            //Write the Override and runOpMode method
            osw.write("\t"+"@Override"+"\n");
            osw.write("\t"+"public void runOpMode() {"+"\n\n");
            osw.write("\t\t"+" int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(\"cameraMonitorViewId\", \"id\", hardwareMap.appContext.getPackageName());"+"\n");
            osw.write("\t\t"+"VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);"+"\n\n");
            osw.write("\t\t"+"parameters.vuforiaLicenseKey = cons.VUFORIA_KEY;"+"\n");
            osw.write("\t\t"+"parameters.cameraDirection   = cons.CAMERA_CHOICE;"+"\n\n");
            osw.write("\t\t"+"//  Instantiate the Vuforia engine"+"\n");
            osw.write("\t\t"+"vuforia = ClassFactory.getInstance().createVuforia(parameters);"+"\n\n");
            osw.write("\t\t"+"targetsSkyStone = this.vuforia.loadTrackablesFromAsset(\"Skystone\");"+"\n");
            osw.write("\t\t"+"//all above lines need to be all autonomous OpMode's runOpMode before initialization"+"\n\n");

            //Write the initialize() line to invoke that method so initialization can be made offline
            osw.write("\t\t"+"initialize();"+"\n\n");

            //Write the waitForStart() line to wait for user input on phone
            osw.write("\t\t"+"waitForStart();"+"\n\n");

            //Write the runCode() line to invoke that method so this can be easily batch run offline
            osw.write("\t\t"+"runCode();"+"\n\n");

            osw.write("\t}\n\n");//closes the runOpMode() method

            //Write the initialize() method that sets the variables for this opMode and initializes the robot HW
            osw.write("\t@Override\n");
            osw.write("\tpublic void initialize() {\n\n");
            //Write the parameter specific constants
            osw.write(String.format("\t\tfoundationPosChange = %.1f;// 0 for moved, 26 for unmoved Foundation\n",fPC));
            osw.write(String.format("\t\tinsideOutside = %.1f;// 0 for Inside, 24 for Outside\n",iO));
            osw.write(String.format("\t\tsideColor = %.1f;// + for Blue, - for Red\n\n",sC));
            //Initialize the HW
            osw.write("\t\t"+"initializeMiniBot();"+"\n\n");
            osw.write("\t}\n\n");//closes the initialize() method

            //Write the runCode() method that contains all the unique method calls
            osw.write("\t@Override\n");
            osw.write("\tpublic void runCode() {\n\n");
            osw.write("\t\t"+"runtime.reset();"+"\n\n");
            osw.write("\t\t"+"Billy.initIMU(this);"+"\n\n");
            if(stoneSide) {
                //Write the opMode for stone side
                osw.write("\t\t" + "fwdToTwoStone();" + "\n\n");
                osw.write("\t\t" + "if(testModeActive){" + "\n");
                osw.write("\t\t\t" + "vuforiaStoneLocateOffline(stoneSelect);" + "\n");
                osw.write("\t\t" + "}" + "\n");
                osw.write("\t\t" + "else{" + "\n");
                osw.write("\t\t\t" + "vuforiaStoneLocate();" + "\n");
                osw.write("\t\t" + "}" + "\n\n");
                osw.write("\t\t" + "goToStone();" + "\n\n");
                osw.write("\t\t" + "takeStone1();" + "\n\n");
                osw.write("\t\t" + "getStone2();" + "\n\n");
                osw.write("\t\t" + "takeStone2();" + "\n\n");
                osw.write("\t\t" + "twoStonePark();" + "\n\n");
            }
            else {
                //Write the opMode for foundation side
                osw.write("\t\t" + "grabFoundation();" + "\n\n");
//                osw.write("\t\t" + "foundationInCorner();" + "\n\n");
                osw.write("\t\t" + "grabAndRotateFoundation();" + "\n\n");

            }

            osw.write("\t\t"+"telemetry.addLine(\"OpMode Complete\");"+"\n");
            osw.write("\t\t"+"telemetry.update();"+"\n");
            osw.write("\t\t"+"sleep(500);"+"\n");
            osw.write("\t}\n\n");//closes the runCode() method

            //Write the closing bracket for the OpMode
            osw.write("}\n");//Closes the class
            osw.close();

        }

        catch (Exception e) {
//            Log.e("Exception", e.toString());
            System.out.println("There was an Error Writing: " + e.toString());
        }
    }
}
