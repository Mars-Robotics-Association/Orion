package org.firstinspires.ftc.teamcode.Core.HermesLog;

import android.util.Log;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Core.HermesLog.DataTypes.Timestamp;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
    Class for sending data over a websocket. To use, call AddDate(yourObjects[]) to add the public
    variables of an object to the queue. It is recommended you create simple data holding classes for
    this. Then, make sure Update() is called every loop of the program.
*/

public class HermesLog
{
    private ElapsedTime runtime = new ElapsedTime();

    private Gson gson;
    private DashboardWebSocketServer server;

    private List<Object> accumulatedData = new ArrayList<Object>();
    boolean accumulateData = false;

    private String tag = "LOG";
    private double updateTime = 500;
    private double lastSendTime = 0;
    private OpMode opMode;

    public HermesLog(String setTag, double updateTimeMs, OpMode currentOpMode){
        //set variables
        tag = setTag;
        updateTime = updateTimeMs;
        gson = new GsonBuilder().create();
        opMode = currentOpMode;
        accumulateData = false;

        accumulatedData.clear();

        //Set up web socket server
        Log.i(tag, "CONFIGURE NANOHTTPD SERVER");
        server = DashboardWebSocketServer.getInstance();
        if ( server == null) {
            server = new DashboardWebSocketServer();
            try {
                server.start(10000, false);
            } catch (IOException e) {
                Log.w("failed to start server", e);
            }
        }
    }

    public void Start() {
        runtime.reset();
    }

    //Sends all data every specified number of milliseconds
    public void Update(){
        if(runtime.milliseconds() >= lastSendTime + updateTime) {
            lastSendTime = runtime.milliseconds();
            SendDataImmediate(accumulatedData.toArray());
            accumulatedData.clear();
        }
        if(!accumulateData) accumulatedData.clear();
    }

    //Adds data to be sent on the next update
    public void AddData(Object[] data){
        if(data == null) return;

        for (int i=0; i<data.length; i++) {
            Object thing = data[i];
            accumulatedData.add(thing);
        }
    }

    //Sends json containing the public member variables from an array of public objects
    public void SendDataImmediate(Object[] data) {
        if(data == null) return;

        //Add a timestamp to the data
        List<Object> dataToSend = new ArrayList<Object>(Arrays.asList(data));
        Timestamp timestamp = new Timestamp((double)runtime.time(TimeUnit.MILLISECONDS));
        dataToSend.add(timestamp);

        //Creates a list of json objects to combine into one json block
        List<JSONObject> jsonObjects = new ArrayList<JSONObject>();
        for (Object obj : dataToSend) {
            String msg = gson.toJson(obj); //convert object to a json string
            try {
                JSONObject jsonObject = new JSONObject(msg); //convert the string to a json object
                jsonObjects.add(jsonObject); //add to the list of json objects

            }catch (JSONException err){
                Log.d("Error", err.toString());
            }

        }

        //Combines the json into one block
        JSONObject finalJson = null;
        try {
            finalJson = CombineJSON(jsonObjects);
        } catch (JSONException e) {
            e.printStackTrace();
        }

        //Convert the json to a string
        String stringToSend = finalJson.toString();

        //Send the final json to the websocket as a string
        opMode.telemetry.addData("Message", stringToSend);
        DashboardWebSocketServer.getInstance().send(stringToSend);
        Log.i(tag, stringToSend);
        opMode.telemetry.update();

    }

    //Combines a list of json objects into a single json object
    public JSONObject CombineJSON(List<JSONObject> data) throws JSONException {
        JSONObject merged = new JSONObject();
        //For each object, iterate through all its keys and add them to the final json object
        for (JSONObject obj : data) {
            Iterator it = obj.keys();
            while (it.hasNext()) {
                String key = (String)it.next();
                merged.put(key, obj.get(key));
            }
        }
        return merged;
    }
}
