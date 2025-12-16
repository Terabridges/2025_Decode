package org.firstinspires.ftc.teamcode.config.utility;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Lightweight table holder for CSV data used by the regression solver.
 */
public class ShooterTable {
    public final double[] p1;
    public final double[] l1;

    public ShooterTable(double[] p1, double[] l1) { this.p1 = p1; this.l1 = l1; }

    public static ShooterTable fromResource(String resourcePath) throws IOException {
        InputStream is = ShooterTable.class.getResourceAsStream(resourcePath);
        if (is == null) throw new IOException("Resource not found: " + resourcePath);
        try (BufferedReader br = new BufferedReader(new InputStreamReader(is, StandardCharsets.UTF_8))) {
            return parse(br);
        }
    }

    public static ShooterTable fromFile(Path path) throws IOException {
        try (BufferedReader br = Files.newBufferedReader(path, StandardCharsets.UTF_8)) {
            return parse(br);
        }
    }

    private static ShooterTable parse(BufferedReader br) throws IOException {
        String line;
        boolean headerSeen = false;
        int colP = -1, colL = -1;
        List<Double> px = new ArrayList<>();
        List<Double> ly = new ArrayList<>();
        while ((line = br.readLine()) != null) {
            line = line.trim();
            if (line.isEmpty()) continue;
            String[] cols = line.split(",");
            for (int i = 0; i < cols.length; i++) cols[i] = cols[i].trim();
            if (!headerSeen) {
                for (int i = 0; i < cols.length; i++) {
                    String c = cols[i].toLowerCase();
                    if (c.equals("p_1") || c.equals("p1") || c.equals("x")) colP = i;
                    if (c.equals("l_1") || c.equals("l1") || c.equals("y")) colL = i;
                }
                if (colP >= 0 && colL >= 0) { headerSeen = true; continue; }
                // no header
                headerSeen = true; colP = 0; colL = 1;
            }
            if (cols.length <= Math.max(colP, colL)) continue;
            try {
                double a = Double.parseDouble(cols[colP]);
                double b = Double.parseDouble(cols[colL]);
                px.add(a); ly.add(b);
            } catch (NumberFormatException ex) {
                // skip
            }
        }
        return new ShooterTable(px.stream().mapToDouble(Double::doubleValue).toArray(), ly.stream().mapToDouble(Double::doubleValue).toArray());
    }
}
