Control Hub Logs UI

How to use
- Connect your computer to the Control Hub WiFi network
- Open your browser and navigate to http://<control-hub-ip>:8081/
- You will see a grid of saved logs and can download any available file

Notes
- Logs are stored under the app external files directory: Android/data/<package>/files/ftc_saved_logs
- The embedded server listens on port 8081 and is started by the robot controller activity on resume
