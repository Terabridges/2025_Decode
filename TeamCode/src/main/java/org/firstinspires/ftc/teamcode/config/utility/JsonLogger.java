package org.firstinspires.ftc.teamcode.config.utility;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import com.qualcomm.robotcore.util.RobotLog;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Map;

/**
 * JsonLogger - small utility to build up a JSON-format log during an OpMode run
 * and write the results into the Android filesystem of the Rev Control Hub.
 */
public class JsonLogger {
	private static final String TAG = "JsonLogger";

	private final Context context;
	private final JSONObject root;
	private final JSONObject metadata;
	private final JSONObject data;
	private final JSONArray events;

	private String sessionName = "session";
	/** Default folder under app external files dir used by JsonLogger and LogWebServer */
	private String baseFolderName = "ftc_saved_logs";
	private String fileName = null; // if null, it is generated from sessionName + timestamp

	private final SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd'T'HH-mm-ss.SSSZ", Locale.US);

	public JsonLogger(Context context) { this(context, "session"); }

	public JsonLogger(Context context, String sessionName) {
		this.context = context.getApplicationContext();
		this.sessionName = sessionName != null ? sessionName : "session";

		root = new JSONObject();
		metadata = new JSONObject();
		data = new JSONObject();
		events = new JSONArray();

		try {
			root.put("sessionName", this.sessionName);
			root.put("timestamp", dateFormat.format(new Date()));
			root.put("metadata", metadata);
			root.put("data", data);
			root.put("events", events);
		} catch (JSONException e) {
			RobotLog.logStacktrace(e);
		}
	}

	public void setSessionName(String name) {
		if (name == null) return;
		this.sessionName = name;
		try { root.put("sessionName", name); } catch (JSONException e) { RobotLog.logStacktrace(e); }
	}

	public void setBaseFolderName(String baseFolder) { if (baseFolder == null) return; this.baseFolderName = baseFolder; }

	public void setFileName(String filename) { this.fileName = filename; }

	public void setMetadata(String key, Object value) {
		try { metadata.put(key, wrap(value)); } catch (JSONException e) { RobotLog.logStacktrace(e); }
	}

	public void addEvent(String eventName, Map<String, Object> eventData) {
		try {
			JSONObject ev = new JSONObject();
			ev.put("name", eventName);
			ev.put("timestamp", dateFormat.format(new Date()));
			if (eventData != null) {
				for (Map.Entry<String, Object> e : eventData.entrySet()) {
					ev.put(e.getKey(), wrap(e.getValue()));
				}
			}
			events.put(ev);
		} catch (JSONException e) {
			RobotLog.logStacktrace(e);
		}
	}

	public void createTable(String tableName) { ensureArrayAtPath(tableName); }

	public void addTableRow(String tableName, Map<String, Object> rowData) {
		JSONArray array = ensureArrayAtPath(tableName);
		JSONObject row = new JSONObject();
		if (rowData != null) {
			for (Map.Entry<String, Object> e : rowData.entrySet()) {
				try { row.put(e.getKey(), wrap(e.getValue())); } catch (JSONException ex) { RobotLog.logStacktrace(ex); }
			}
		}
		array.put(row);
	}

	public JSONArray ensureArrayAtPath(String path) {
		if (path == null) path = "unnamed";
		String[] parts = path.split("/");
		JSONObject current = data;
		try {
			for (int i = 0; i < parts.length - 1; i++) {
				String p = parts[i];
				if (!current.has(p) || current.isNull(p) || !(current.get(p) instanceof JSONObject)) {
					JSONObject child = new JSONObject();
					current.put(p, child);
					current = child;
				} else { current = current.getJSONObject(p); }
			}
			String last = parts[parts.length - 1];
			if (!current.has(last) || current.isNull(last) || !(current.get(last) instanceof JSONArray)) {
				JSONArray arr = new JSONArray();
				current.put(last, arr);
				return arr;
			} else { return current.getJSONArray(last); }
		} catch (JSONException e) {
			RobotLog.logStacktrace(e);
			return new JSONArray();
		}
	}

	public void addToList(String path, Object value) { JSONArray arr = ensureArrayAtPath(path); arr.put(wrap(value)); }

	public void set(String path, Object value) {
		if (path == null) return;
		String[] parts = path.split("/");
		JSONObject current = data;
		try {
			for (int i = 0; i < parts.length - 1; i++) {
				String p = parts[i];
				if (!current.has(p) || current.isNull(p) || !(current.get(p) instanceof JSONObject)) {
					JSONObject child = new JSONObject(); current.put(p, child); current = child;
				} else { current = current.getJSONObject(p); }
			}
			String last = parts[parts.length - 1]; current.put(last, wrap(value));
		} catch (JSONException e) { RobotLog.logStacktrace(e); }
	}

	public File save() {
		ensureRootTimestamp(); File out = null;
		try {
			File dir = getLogsDirectory(); if (!dir.exists()) { boolean ok = dir.mkdirs(); if (!ok) RobotLog.vv(TAG, "Could not create log directory: %s", dir.getAbsolutePath()); }
			String fn = fileName; if (fn == null) { fn = safeFileName(sessionName) + "_" + dateFormat.format(new Date()) + ".json"; }
			out = new File(dir, fn);
			try (FileOutputStream fos = new FileOutputStream(out)) {
				byte[] bytes = getJSONString().getBytes("UTF-8"); fos.write(bytes); fos.getFD().sync();
			}

			RobotLog.vv(TAG, "Saved json log to %s", out.getAbsolutePath());
		} catch (Exception e) { RobotLog.logStacktrace(e); Log.e(TAG, "Failed to save JSON log", e); }
		return out;
	}

		/** Helper: returns the logs directory used by JsonLogger on this device */
		public static File getLogsDirectory(Context ctx) {
			File baseDir = null;
			try {
				baseDir = ctx.getExternalFilesDir("ftc_saved_logs");
				if (baseDir == null) baseDir = new File(Environment.getExternalStorageDirectory(), "ftc_saved_logs");
			} catch (Exception e) { RobotLog.logStacktrace(e); }
			if (baseDir == null) baseDir = new File(".");
			return baseDir;
		}

	public String getJSONString() { try { return root.toString(2); } catch (JSONException e) { RobotLog.logStacktrace(e); return root.toString(); } }

	private void ensureRootTimestamp() { try { root.put("savedAt", dateFormat.format(new Date())); } catch (JSONException e) { RobotLog.logStacktrace(e); } }

	private File getLogsDirectory() {
		File baseDir = null;
		try {
			baseDir = context.getExternalFilesDir(baseFolderName);
			if (baseDir == null) baseDir = new File(Environment.getExternalStorageDirectory(), baseFolderName);
		} catch (Exception e) { RobotLog.logStacktrace(e); }
		if (baseDir == null) baseDir = new File(".");
		return baseDir;
	}

	private static String safeFileName(String s) { if (s == null) return "session"; return s.replaceAll("[^a-zA-Z0-9-_]", "_"); }

	private static Object wrap(Object o) { if (o == null) return JSONObject.NULL; try { return JSONObject.wrap(o); } catch (Throwable t) { return o.toString(); } }

}