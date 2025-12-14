package org.firstinspires.ftc.robotcontroller.internal;

import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Very small HTTP server to serve a static front-end and provide APIs to list and download logs
 * placed under the app external files dir subfolder `ftc_saved_logs`.
 */
public class LogWebServer {
    private static final String TAG = "LogWebServer";

    private final Context context;
    private ServerSocket serverSocket;
    private volatile boolean running = false;
    private Thread acceptThread;
    private int port = 8081;

    public LogWebServer(Context context, int port) {
        this.context = context.getApplicationContext();
        this.port = port;
    }

    public synchronized void start() {
        if (running) return;
        running = true;
        acceptThread = new Thread(this::acceptLoop, "LogWebServer-Accept");
        acceptThread.start();
        RobotLog.vv(TAG, "LogWebServer started on port %d", port);
    }

    public synchronized void stop() {
        running = false;
        try {
            if (serverSocket != null) serverSocket.close();
        } catch (IOException e) { RobotLog.logStacktrace(e); }
        if (acceptThread != null) acceptThread.interrupt();
        RobotLog.vv(TAG, "LogWebServer stopped");
    }

    private void acceptLoop() {
        try (ServerSocket ss = new ServerSocket(port)) {
            this.serverSocket = ss;
            while (running && !ss.isClosed()) {
                Socket client = ss.accept();
                Thread t = new Thread(() -> handleClient(client));
                t.start();
            }
        } catch (IOException e) {
            if (running) RobotLog.logStacktrace(e);
        }
    }

    private void handleClient(Socket client) {
        try (Socket socket = client;
             InputStream in = new BufferedInputStream(socket.getInputStream());
             OutputStream out = new BufferedOutputStream(socket.getOutputStream())) {

            // Very small and forgiving HTTP parser: read the request line
            String requestLine = readLine(in);
            if (requestLine == null || requestLine.isEmpty()) return;
            String[] parts = requestLine.split(" ");
            if (parts.length < 2) return;
            String method = parts[0];
            String path = parts[1];

            // Drain headers
            String line;
            while ((line = readLine(in)) != null) {
                if (line.isEmpty()) break;
            }

            // Route
            if (!"GET".equals(method)) {
                writeResponse(out, 405, "text/plain", "Method Not Allowed".getBytes(StandardCharsets.UTF_8));
                return;
            }

            if (path.equals("/") || path.equals("/index.html")) {
                byte[] html = loadAsset("www/index.html");
                if (html != null) writeResponse(out, 200, "text/html; charset=utf-8", html);
                else writeResponse(out, 404, "text/plain", "Not Found".getBytes(StandardCharsets.UTF_8));
                return;
            }

            if (path.startsWith("/app.js")) {
                byte[] js = loadAsset("www/app.js");
                if (js != null) writeResponse(out, 200, "application/javascript; charset=utf-8", js);
                else writeResponse(out, 404, "text/plain", "Not Found".getBytes(StandardCharsets.UTF_8));
                return;
            }

            if (path.startsWith("/style.css")) {
                byte[] css = loadAsset("www/style.css");
                if (css != null) writeResponse(out, 200, "text/css; charset=utf-8", css);
                else writeResponse(out, 404, "text/plain", "Not Found".getBytes(StandardCharsets.UTF_8));
                return;
            }

            if (path.startsWith("/api/list")) {
                JSONArray arr = listLogs();
                String s = arr.toString();
                writeResponse(out, 200, "application/json; charset=utf-8", s.getBytes(StandardCharsets.UTF_8));
                return;
            }

            if (path.startsWith("/api/preview")) {
                // Expect /api/preview?file=filename
                String fileParam = null;
                int q = path.indexOf('?');
                if (q >= 0 && q + 1 < path.length()) {
                    String query = path.substring(q + 1);
                    for (String kv : query.split("&")) {
                        String[] kvp = kv.split("=");
                        if (kvp.length == 2 && kvp[0].equals("file")) {
                            fileParam = URLDecoder.decode(kvp[1], "UTF-8");
                        }
                    }
                }
                if (fileParam == null) {
                    writeResponse(out, 400, "text/plain", "Missing file parameter".getBytes(StandardCharsets.UTF_8));
                    return;
                }
                File f = getLogFile(fileParam);
                if (f == null || !f.exists()) {
                    writeResponse(out, 404, "text/plain", "File not found".getBytes(StandardCharsets.UTF_8));
                    return;
                }
                // Return first N bytes as text/plain
                final int MAX_PREVIEW = 64 * 1024; // 64KB
                try (FileInputStream fis = new FileInputStream(f)) {
                    byte[] buf = new byte[MAX_PREVIEW];
                    int r = fis.read(buf);
                    if (r < 0) r = 0;
                    byte[] outBytes = new byte[r];
                    System.arraycopy(buf, 0, outBytes, 0, r);
                    writeResponse(out, 200, "text/plain; charset=utf-8", outBytes);
                } catch (IOException e) {
                    RobotLog.logStacktrace(e);
                    writeResponse(out, 500, "text/plain", "Failed to read file".getBytes(StandardCharsets.UTF_8));
                }
                return;
            }

            if (path.startsWith("/download")) {
                // Expect /download?file=filename
                String fileParam = null;
                int q = path.indexOf('?');
                if (q >= 0 && q + 1 < path.length()) {
                    String query = path.substring(q + 1);
                    for (String kv : query.split("&")) {
                        String[] kvp = kv.split("=");
                        if (kvp.length == 2 && kvp[0].equals("file")) {
                            fileParam = URLDecoder.decode(kvp[1], "UTF-8");
                        }
                    }
                }
                if (fileParam == null) {
                    writeResponse(out, 400, "text/plain", "Missing file parameter".getBytes(StandardCharsets.UTF_8));
                    return;
                }
                File f = getLogFile(fileParam);
                if (f == null || !f.exists()) {
                    writeResponse(out, 404, "text/plain", "File not found".getBytes(StandardCharsets.UTF_8));
                    return;
                }
                // Serve file bytes with Content-Disposition for download
                try (FileInputStream fis = new FileInputStream(f)) {
                    String header = "HTTP/1.1 200 OK\r\n" +
                            "Content-Type: application/octet-stream\r\n" +
                            "Content-Length: " + f.length() + "\r\n" +
                            "Content-Disposition: attachment; filename=\"" + f.getName() + "\"\r\n" +
                            "Connection: close\r\n\r\n";
                    out.write(header.getBytes(StandardCharsets.UTF_8));
                    byte[] buf = new byte[8192];
                    int r;
                    while ((r = fis.read(buf)) > 0) out.write(buf, 0, r);
                    out.flush();
                }
                return;
            }

            writeResponse(out, 404, "text/plain", "Not Found".getBytes(StandardCharsets.UTF_8));

        } catch (IOException e) {
            RobotLog.logStacktrace(e);
        }
    }

    private byte[] loadAsset(String assetPath) {
        try {
            AssetManager am = context.getAssets();
            try (InputStream is = am.open(assetPath)) {
                // Avoid relying on InputStream.readAllBytes for compatibility
                java.io.ByteArrayOutputStream baos = new java.io.ByteArrayOutputStream();
                byte[] buf = new byte[4096];
                int r;
                while ((r = is.read(buf)) != -1) baos.write(buf, 0, r);
                return baos.toByteArray();
            }
        } catch (IOException e) {
            RobotLog.logStacktrace(e);
            return null;
        }
    }

    private JSONArray listLogs() {
        JSONArray arr = new JSONArray();
        try {
            File dir = context.getExternalFilesDir("ftc_saved_logs");
            if (dir == null || !dir.exists()) return arr;
            File[] files = dir.listFiles();
            if (files == null) return arr;
            SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss", Locale.US);
            String host = ""; // relative URLs
            for (File f : files) {
                if (!f.isFile()) continue;
                JSONObject o = new JSONObject();
                o.put("name", f.getName());
                o.put("size", f.length());
                o.put("modified", sdf.format(new Date(f.lastModified())));
                o.put("url", "/download?file=" + java.net.URLEncoder.encode(f.getName(), "UTF-8"));
                arr.put(o);
            }
        } catch (Exception e) { RobotLog.logStacktrace(e); }
        return arr;
    }

    private File getLogFile(String filename) {
        try {
            File dir = context.getExternalFilesDir("ftc_saved_logs");
            if (dir == null) return null;
            File f = new File(dir, filename);
            // Avoid path traversal
            String base = dir.getCanonicalPath();
            String cand = f.getCanonicalPath();
            if (!cand.startsWith(base)) return null;
            return f;
        } catch (IOException e) { RobotLog.logStacktrace(e); return null; }
    }

    private String readLine(InputStream in) throws IOException {
        StringBuilder sb = new StringBuilder();
        int c;
        boolean gotCR = false;
        while ((c = in.read()) != -1) {
            if (c == '\r') { gotCR = true; continue; }
            if (c == '\n') break;
            sb.append((char)c);
        }
        return sb.toString();
    }

    private void writeResponse(OutputStream out, int statusCode, String contentType, byte[] body) throws IOException {
        String statusText = (statusCode == 200) ? "OK" : Integer.toString(statusCode);
        String header = "HTTP/1.1 " + statusCode + " " + statusText + "\r\n" +
                "Content-Type: " + contentType + "\r\n" +
                "Content-Length: " + body.length + "\r\n" +
                "Connection: close\r\n\r\n";
        out.write(header.getBytes(StandardCharsets.UTF_8));
        out.write(body);
        out.flush();
    }
}
