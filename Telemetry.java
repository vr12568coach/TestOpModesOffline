package TestOpModesOffline;

/**
 * Created by Spiessbach on 8/26/2018.
 */

public class Telemetry {

//        telemetry.addData("Distance", " Command(%.2f), Current(%.2f)", commandInch, distanceTraveledInch);
//        telemetry.addData("control loop:", "Current Error(%.1f), Sum Error(%.1f), Steering Power(%.1f)", error, sumError, steerInput);
//        telemetry.update();
    public String[] TMData = new String[200];
    public int index;



    public void addData(String caption, String format, Object... args){
        StringBuilder TelemetryString = new StringBuilder();
        TelemetryString.append(caption);
        TelemetryString.append(": ");
        TelemetryString.append(String.format(format,args));

        TMData[index] =  TelemetryString.toString();
        index +=1;
    }
    public void addLine(String caption){
        StringBuilder TelemetryString = new StringBuilder();
        TelemetryString.append(caption);
//        TelemetryString.append(": ");
//        TelemetryString.append(args.toString());

        TMData[index] =  TelemetryString.toString();
        index +=1;
    }
    public String[] update(){
        for(int i = 0;i<index;i++) {
            System.out.println(TMData[i]);
        }
            index = 0;
        return TMData;
    }

}
