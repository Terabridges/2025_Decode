package org.firstinspires.ftc.teamcode.psikit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.psilynx.psikit.ftc.wrappers.GamepadWrapper;

import java.io.File;
import java.io.IOException;
import java.lang.annotation.Annotation;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Enumeration;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Scanner;
import java.util.Set;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

/**
 * Minimal PC/CLI entrypoint for replaying an FTC {@link LinearOpMode} using PsiKit.
 *
 * This does NOT use JUnit. It is intended to be run from Gradle as a JavaExec task:
 *
 *   ./gradlew :TeamCode:runPsiKitReplay --args="--opmode <fqcn> --log <path-to.rlog>"
 *
 * Notes:
 * - PsiKit replay is configured via system properties (same ones the tests use).
 * - Some OpModes may still require Robolectric/Android stubs depending on what they touch.
 */
public final class PsiKitReplayCliMain {

    private static final String LOG_PATH_PROPERTY = "psikitReplayLog";
    private static final String OPMODE_PROPERTY = "psikitReplayOpMode";
    private static final String WRITE_OUTPUT_PROPERTY = "psikitReplayWriteOutput";
    private static final String OUTPUT_DIR_PROPERTY = "psikitReplayOutputDir";

    private static final String DEFAULT_OPMODE_SCAN_PACKAGE = "org.firstinspires.ftc.teamcode";

    private PsiKitReplayCliMain() {
    }

    public static void main(String[] args) throws Exception {
        CliArgs cli = CliArgs.parse(args);
        if (cli.showHelp) {
            printUsage();
            return;
        }

        if (cli.logPath != null) {
            System.setProperty(LOG_PATH_PROPERTY, cli.logPath.toAbsolutePath().toString());
        }
        if (cli.opModeClassName != null) {
            System.setProperty(OPMODE_PROPERTY, cli.opModeClassName);
        }
        if (cli.writeOutput != null) {
            System.setProperty(WRITE_OUTPUT_PROPERTY, String.valueOf(cli.writeOutput));
        }
        if (cli.outputDir != null) {
            System.setProperty(OUTPUT_DIR_PROPERTY, cli.outputDir.toAbsolutePath().toString());
        }

        try (Scanner scanner = new Scanner(System.in)) {
            if (System.getProperty(OPMODE_PROPERTY) == null || System.getProperty(OPMODE_PROPERTY).trim().isEmpty()) {
                List<Class<? extends LinearOpMode>> opModes = findLinearOpModes(DEFAULT_OPMODE_SCAN_PACKAGE);
                if (opModes.isEmpty()) {
                    throw new IllegalStateException(
                            "No LinearOpModes found under package '" + DEFAULT_OPMODE_SCAN_PACKAGE + "'. " +
                                    "Pass --opmode <fully.qualified.ClassName>."
                    );
                }

                System.out.println("Available LinearOpModes (" + opModes.size() + "):");
                for (int i = 0; i < opModes.size(); i++) {
                    Class<? extends LinearOpMode> c = opModes.get(i);
                    System.out.printf(Locale.US, "  [%d] %s%s%n", i, c.getName(), formatOpModeAnnotations(c));
                }
                System.out.print("Select an opmode index: ");
                int idx = Integer.parseInt(scanner.nextLine().trim());
                if (idx < 0 || idx >= opModes.size()) {
                    throw new IllegalArgumentException("Index out of range: " + idx);
                }
                System.setProperty(OPMODE_PROPERTY, opModes.get(idx).getName());
            }

            if (System.getProperty(LOG_PATH_PROPERTY) == null || System.getProperty(LOG_PATH_PROPERTY).trim().isEmpty()) {
                List<Path> candidates = findRlogCandidates(Paths.get("."));
                if (!candidates.isEmpty()) {
                    System.out.println("Found .rlog candidates in current folder:");
                    for (int i = 0; i < candidates.size(); i++) {
                        System.out.printf(Locale.US, "  [%d] %s%n", i, candidates.get(i).toAbsolutePath());
                    }
                    System.out.print("Select a log index (or press Enter to type a path): ");
                    String line = scanner.nextLine().trim();
                    if (!line.isEmpty()) {
                        int idx = Integer.parseInt(line);
                        if (idx < 0 || idx >= candidates.size()) {
                            throw new IllegalArgumentException("Index out of range: " + idx);
                        }
                        System.setProperty(LOG_PATH_PROPERTY, candidates.get(idx).toAbsolutePath().toString());
                    }
                }

                if (System.getProperty(LOG_PATH_PROPERTY) == null || System.getProperty(LOG_PATH_PROPERTY).trim().isEmpty()) {
                    System.out.print("Enter path to .rlog file: ");
                    String pathStr = scanner.nextLine().trim();
                    if (pathStr.isEmpty()) {
                        throw new IllegalArgumentException("Missing --log (or psikitReplayLog)");
                    }
                    System.setProperty(LOG_PATH_PROPERTY, Paths.get(pathStr).toAbsolutePath().toString());
                }
            }
        }

        String opModeClassName = System.getProperty(OPMODE_PROPERTY).trim();
        Path logPath = Paths.get(System.getProperty(LOG_PATH_PROPERTY)).toAbsolutePath();

        if (!Files.exists(logPath)) {
            throw new IllegalArgumentException("Log file does not exist: " + logPath);
        }

        System.out.println("[PsiKitReplay] opmode=" + opModeClassName);
        System.out.println("[PsiKitReplay] log=" + logPath);
        System.out.println("[PsiKitReplay] writeOutput=" + System.getProperty(WRITE_OUTPUT_PROPERTY));
        System.out.println("[PsiKitReplay] outputDir=" + System.getProperty(OUTPUT_DIR_PROPERTY));

        LinearOpMode opMode = instantiateLinearOpMode(opModeClassName);

        // Minimal wiring consistent with our replay unit tests.
        // Many OpModes will work in replay if they tolerate null hardwareMap.
        opMode.hardwareMap = null;
        opMode.gamepad1 = new GamepadWrapper(null);
        opMode.gamepad2 = new GamepadWrapper(null);

        opMode.runOpMode();

        System.out.println("[PsiKitReplay] done");
    }

    private static void printUsage() {
        System.out.println("PsiKitReplay CLI (PC runner)");
        System.out.println();
        System.out.println("Usage:");
        System.out.println("  --opmode <fully.qualified.LinearOpModeClass>");
        System.out.println("  --log <path-to.rlog>");
        System.out.println("Optional:");
        System.out.println("  --write-output true|false   (sets -D" + WRITE_OUTPUT_PROPERTY + ")");
        System.out.println("  --output-dir <path>         (sets -D" + OUTPUT_DIR_PROPERTY + ")");
        System.out.println("  --help");
        System.out.println();
        System.out.println("If --opmode or --log are omitted, an interactive prompt will appear.");
    }

    private static LinearOpMode instantiateLinearOpMode(String opModeClassName) throws Exception {
        Class<?> clazz = Class.forName(opModeClassName);
        if (!LinearOpMode.class.isAssignableFrom(clazz)) {
            throw new IllegalArgumentException("Class does not extend LinearOpMode: " + opModeClassName);
        }
        try {
            return (LinearOpMode) clazz.getDeclaredConstructor().newInstance();
        } catch (NoSuchMethodException noNoArgCtor) {
            throw new IllegalArgumentException(
                    "OpMode must have a public no-arg constructor for CLI replay: " + opModeClassName,
                    noNoArgCtor
            );
        }
    }

    private static String formatOpModeAnnotations(Class<?> c) {
        String name = getOpModeDisplayName(c);
        String kind = getOpModeKind(c);
        if (name == null && kind == null) {
            return "";
        }
        if (name == null) {
            return " (" + kind + ")";
        }
        if (kind == null) {
            return " (\"" + name + "\")";
        }
        return " (\"" + name + "\"; " + kind + ")";
    }

    private static String getOpModeKind(Class<?> c) {
        if (c.getAnnotation(TeleOp.class) != null) {
            return "@TeleOp";
        }
        if (c.getAnnotation(Autonomous.class) != null) {
            return "@Autonomous";
        }
        return null;
    }

    private static String getOpModeDisplayName(Class<?> c) {
        Annotation teleop = c.getAnnotation(TeleOp.class);
        if (teleop != null) {
            return ((TeleOp) teleop).name();
        }
        Annotation auto = c.getAnnotation(Autonomous.class);
        if (auto != null) {
            return ((Autonomous) auto).name();
        }
        return null;
    }

    private static List<Path> findRlogCandidates(Path dir) throws IOException {
        if (dir == null || !Files.isDirectory(dir)) {
            return Collections.emptyList();
        }
        List<Path> out = new ArrayList<>();
        try (java.util.stream.Stream<Path> stream = Files.list(dir)) {
            stream
                    .filter(p -> p.getFileName().toString().toLowerCase(Locale.US).endsWith(".rlog"))
                    .sorted(Comparator.comparing(Path::toString))
                    .forEach(out::add);
        }
        return out;
    }

    /**
     * Best-effort classpath scanner.
     * - Works well when running from Gradle/IDE where classes are directories.
     * - Also supports jar entries.
     */
    @SuppressWarnings("unchecked")
    private static List<Class<? extends LinearOpMode>> findLinearOpModes(String rootPackage) throws IOException {
        String path = rootPackage.replace('.', '/');
        ClassLoader cl = Thread.currentThread().getContextClassLoader();
        if (cl == null) {
            cl = PsiKitReplayCliMain.class.getClassLoader();
        }

        Set<String> classNames = new HashSet<>();
        Enumeration<java.net.URL> resources = cl.getResources(path);
        while (resources.hasMoreElements()) {
            java.net.URL url = resources.nextElement();
            String protocol = url.getProtocol();
            if ("file".equals(protocol)) {
                Path baseDir;
                try {
                    baseDir = Paths.get(url.toURI());
                } catch (Exception e) {
                    // Fallback for odd URL formats.
                    String decoded = URLDecoder.decode(url.getPath(), StandardCharsets.UTF_8);
                    if (decoded.matches("^/[A-Za-z]:/.*")) {
                        decoded = decoded.substring(1);
                    }
                    baseDir = Paths.get(decoded);
                }
                if (Files.isDirectory(baseDir)) {
                    collectClassNamesFromDirectory(baseDir, rootPackage, classNames);
                }
            } else if ("jar".equals(protocol)) {
                // Example: jar:file:/path/to.jar!/org/firstinspires/...
                String spec = url.toString();
                int bang = spec.indexOf('!');
                int jarPrefix = spec.indexOf("jar:");
                if (bang > 0 && jarPrefix == 0) {
                    String jarPath = spec.substring("jar:".length(), bang);
                    if (jarPath.startsWith("file:")) {
                        jarPath = jarPath.substring("file:".length());
                    }
                    jarPath = URLDecoder.decode(jarPath, StandardCharsets.UTF_8);
                    collectClassNamesFromJar(Paths.get(jarPath), path + "/", classNames);
                }
            }
        }

        List<Class<? extends LinearOpMode>> out = new ArrayList<>();
        for (String cn : classNames) {
            try {
                Class<?> c = Class.forName(cn, false, cl);
                if (c.isInterface() || java.lang.reflect.Modifier.isAbstract(c.getModifiers())) {
                    continue;
                }
                if (!LinearOpMode.class.isAssignableFrom(c)) {
                    continue;
                }
                // Ensure it can be constructed.
                c.getDeclaredConstructor();
                out.add((Class<? extends LinearOpMode>) c);
            } catch (Throwable ignored) {
                // Skip classes that fail to load in this environment.
            }
        }

        out.sort(Comparator.comparing(Class::getName));
        return out;
    }

    private static void collectClassNamesFromDirectory(Path baseDir, String rootPackage, Set<String> out) throws IOException {
        Files.walk(baseDir)
                .filter(p -> p.getFileName().toString().endsWith(".class"))
                .forEach(p -> {
                    Path rel = baseDir.relativize(p);
                    String classFile = rel.toString().replace(File.separatorChar, '.');
                    if (!classFile.endsWith(".class")) {
                        return;
                    }
                    String simple = classFile.substring(0, classFile.length() - ".class".length());
                    if (simple.contains("$")) {
                        // Skip inner/anonymous classes.
                        return;
                    }
                    out.add(rootPackage + "." + simple);
                });
    }

    private static void collectClassNamesFromJar(Path jarPath, String packagePathPrefix, Set<String> out) throws IOException {
        if (!Files.exists(jarPath)) {
            return;
        }
        try (JarFile jar = new JarFile(jarPath.toFile())) {
            Enumeration<JarEntry> entries = jar.entries();
            while (entries.hasMoreElements()) {
                JarEntry e = entries.nextElement();
                String name = e.getName();
                if (!name.startsWith(packagePathPrefix) || !name.endsWith(".class")) {
                    continue;
                }
                if (name.contains("$")) {
                    continue;
                }
                String cn = name.substring(0, name.length() - ".class".length()).replace('/', '.');
                out.add(cn);
            }
        }
    }

    private static final class CliArgs {
        final boolean showHelp;
        final String opModeClassName;
        final Path logPath;
        final Boolean writeOutput;
        final Path outputDir;

        private CliArgs(boolean showHelp, String opModeClassName, Path logPath, Boolean writeOutput, Path outputDir) {
            this.showHelp = showHelp;
            this.opModeClassName = opModeClassName;
            this.logPath = logPath;
            this.writeOutput = writeOutput;
            this.outputDir = outputDir;
        }

        static CliArgs parse(String[] args) {
            if (args == null) {
                return new CliArgs(false, null, null, null, null);
            }

            boolean help = false;
            String opmode = null;
            Path log = null;
            Boolean writeOutput = null;
            Path outputDir = null;

            for (int i = 0; i < args.length; i++) {
                String a = args[i];
                if (a == null) {
                    continue;
                }
                switch (a) {
                    case "--help":
                    case "-h":
                        help = true;
                        break;
                    case "--opmode":
                        opmode = requireValue(args, ++i, "--opmode");
                        break;
                    case "--log":
                        log = Paths.get(requireValue(args, ++i, "--log"));
                        break;
                    case "--write-output":
                        writeOutput = Boolean.parseBoolean(requireValue(args, ++i, "--write-output"));
                        break;
                    case "--output-dir":
                        outputDir = Paths.get(requireValue(args, ++i, "--output-dir"));
                        break;
                    default:
                        // Allow passing a bare .rlog path as a convenience.
                        if (a.toLowerCase(Locale.US).endsWith(".rlog") && log == null) {
                            log = Paths.get(a);
                        } else if (opmode == null && a.contains(".")) {
                            // Allow passing bare FQCN without flag.
                            opmode = a;
                        } else {
                            throw new IllegalArgumentException("Unknown arg: " + a);
                        }
                }
            }

            return new CliArgs(help, opmode, log, writeOutput, outputDir);
        }

        private static String requireValue(String[] args, int idx, String flag) {
            if (idx < 0 || idx >= args.length) {
                throw new IllegalArgumentException("Missing value for " + flag);
            }
            String v = args[idx];
            if (v == null || v.trim().isEmpty()) {
                throw new IllegalArgumentException("Missing value for " + flag);
            }
            return v.trim();
        }
    }
}
