package org.firstinspires.ftc.teamcode.util;

import java.io.*;
import java.net.*;

public class SocketServer {
    String address;
    int port;

    Socket s;
    OutputStreamWriter out;

    public SocketServer(String address, int port) throws IOException {
        this.address = address;
        this.port = port;
    }

    public void init() throws IOException {
        s = new Socket(InetAddress.getByName(address), 6969);
        out = new OutputStreamWriter(s.getOutputStream());
    }

    public void sendValue(String str) throws IOException {
        init();

        out.write(str);
        out.flush();
    }

    public void sendValue(double d) throws IOException {
        init();

        out.write(Double.toString(d));
        out.flush();
    }
}
