package org.firstinspires.ftc.teamcode.Core.HermesLog;

import java.util.ArrayList;
import java.util.List;

import fi.iki.elonen.NanoWSD;

/**
 * WebSocket server that handles dashboard client connections.
 */
public class DashboardWebSocketServer extends NanoWSD {
    private static final int PORT = 8001;

    private static DashboardWebSocketServer instance = null;
    private final List<DashboardWebSocket> sockets = new ArrayList<>();

    public static DashboardWebSocketServer getInstance() {
        return instance;
    }

    DashboardWebSocketServer() {
        super(PORT);
        instance = this;
    }

    void addSocket(DashboardWebSocket socket) {
        synchronized (sockets) {
            sockets.add(socket);
        }
    }

    void removeSocket(DashboardWebSocket socket){
        synchronized (sockets) {
            sockets.remove(socket);
        }
    }

    @Override
    protected WebSocket openWebSocket(IHTTPSession handshake) {
        return new DashboardWebSocket(handshake);
    }

    public void send(String msg) {
        for (DashboardWebSocket ws : sockets) {
            synchronized(sockets) {
                ws.send(msg);
            }
        }
    }
}