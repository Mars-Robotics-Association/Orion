package org.firstinspires.ftc.teamcode.Core.Websocket;

import android.util.Log;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonParser;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.json.JSONException;
import org.json.JSONObject;

import java.io.IOException;
import java.sql.Time;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.TimeUnit;

/*
    Class for sending data over a websocket
*/

class HermesLog
{
    private ElapsedTime runtime = new ElapsedTime();

    private Gson gson;
    private DashboardWebSocketServer server;

    private List<Object> accumulatedMessages = new ArrayList<Object>();

    private String tag = "LOG";
    private double updateTimeMs = 500;
    private double lastSendTime = 0;
    private OpMode opMode;

    public void Init(String setTag, double setUpdateTimeMs, OpMode currentOpMode){
        tag = setTag;
        updateTimeMs = setUpdateTimeMs;
        gson = new GsonBuilder().create();
        opMode = currentOpMode;

        accumulatedMessages.clear();

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

    //Sends json containing the public member variables from an array of public objects
    public void SendData(Object[] data) {
        if(data == null) return;

        List<Object> dataToSend = new ArrayList<Object>(Arrays.asList(data));
        Timestamp timestamp = new Timestamp((double)runtime.time(TimeUnit.MILLISECONDS));
        dataToSend.add(timestamp);

        if(runtime.milliseconds() >= lastSendTime + updateTimeMs) {
            lastSendTime = runtime.milliseconds();
            List<JSONObject> jsonObjects = new ArrayList<JSONObject>();
            for (Object obj : dataToSend) {
                String msg = gson.toJson(obj);
                try {
                    JSONObject jsonObject = new JSONObject(msg);
                    jsonObjects.add(jsonObject);

                }catch (JSONException err){
                    Log.d("Error", err.toString());
                }

            }

            JSONObject finalJson = null;
            try {
                finalJson = CombineJSON(jsonObjects);
            } catch (JSONException e) {
                e.printStackTrace();
            }

            String stringToSend = finalJson.toString();

            opMode.telemetry.addData("Message", stringToSend);
            DashboardWebSocketServer.getInstance().send(stringToSend);
            Log.i(tag, stringToSend);
            opMode.telemetry.update();

        }

    }

    public JSONObject CombineJSON(List<JSONObject> data) throws JSONException {
        JSONObject merged = new JSONObject();
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

    /*public void SendData(Object[] data) {
        List<Object> dataToSend = new ArrayList<Object>(Arrays.asList(data));
        Timestamp timestamp = new Timestamp((double)runtime.time(TimeUnit.MILLISECONDS));
        dataToSend.add(timestamp);

        opMode.telemetry.addData("Runtime", runtime.time(TimeUnit.MILLISECONDS) + " milliseconds");
        if(runtime.milliseconds() >= lastSendTime + updateTimeMs) {
            lastSendTime = runtime.milliseconds();
            for (Object obj : dataToSend) {
                String msg = gson.toJson(obj);
                opMode.telemetry.addData("Message", msg);
                DashboardWebSocketServer.getInstance().send(msg);
                Log.i(tag, msg);
            }
        }
        opMode.telemetry.update();

    }*/
