package org.firstinspires.ftc.teamcode.Core.HermesLog;

import android.util.Log;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;
import fi.iki.elonen.NanoWSD;

/**
 * WebSocket connection to a dashboard client.
 */
public class DashboardWebSocket extends NanoWSD.WebSocket {
    public static final String TAG = "DashboardWebSocket";

    private static final boolean DEBUG = true;

    DashboardWebSocket(NanoHTTPD.IHTTPSession handshakeRequest) {
        super(handshakeRequest);
    }

    @Override
    protected void onOpen() {
        if (DEBUG) {
            String ipAddr = getHandshakeRequest().getRemoteIpAddress();
            Log.i(TAG, "[OPEN]\t" + ipAddr);
        }
        DashboardWebSocketServer.getInstance().addSocket(this);
    }

    @Override
    protected void onClose(NanoWSD.WebSocketFrame.CloseCode code,
                           String reason, boolean initiatedByRemote) {
        if (DEBUG) {
            String ipAddr = getHandshakeRequest().getRemoteIpAddress();
            Log.i(TAG, "[CLOSE]\t" + ipAddr);
        }
        DashboardWebSocketServer.getInstance().removeSocket(this);
    }

    @Override
    protected void onMessage(NanoWSD.WebSocketFrame message) {
        Log.i(TAG, message.getTextPayload());
    }

    @Override
    protected void onPong(NanoWSD.WebSocketFrame pong) {
    }

    @Override
    protected void onException(IOException exception) {
    }

    public void send(String msg) {
        try {
            super.send(msg);
        } catch (IOException e) {
            Log.w(TAG, e);
        }
    }

}
