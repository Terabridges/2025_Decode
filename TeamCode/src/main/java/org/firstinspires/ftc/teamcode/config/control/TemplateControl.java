package org.firstinspires.ftc.teamcode.config.control;

import org.firstinspires.ftc.teamcode.config.subsystems.TemplateSubsystem;
import org.firstinspires.ftc.teamcode.config.utility.EdgeDetector;

public class TemplateControl implements Control {

    //---------------- Software ----------------
    TemplateSubsystem template;
    GamepadView gp1;
    GamepadView gp2;
    EdgeDetector setServoRE = new EdgeDetector(() -> template.setServoOn());

    //---------------- Constructor ----------------
    public TemplateControl(TemplateSubsystem template, GamepadView gp1, GamepadView gp2){
        this.template = template;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    //---------------- Methods ----------------


    //---------------- Interface Methods ----------------
    @Override
    public void update(){
        setServoRE.update(gp1.a());
    }

    @Override
    public void addTelemetry(TelemetrySink telemetry){

    }
}
