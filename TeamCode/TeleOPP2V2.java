package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOPP2V2ok")
public class TeleOPP2V2 extends LinearOpMode {

    private HuskyLens Husky;
    private DcMotor Shooter;
    private DcMotor FrontLeft;
    private DcMotor BackLeft;
    private DcMotor FrontRight;
    private DcMotor BackRight;
    private DcMotor Index;
    private DcMotor InTake;
    private CRServo Torreta;
    private CRServo Angulo; 

    private double MultiplicadorVelocidad = 0.6;
    
    private boolean autoShooterActivo = false;
    private boolean trackingTorretaActivo = false;
    private boolean macroDisparoActivo = false;
    
    private ElapsedTime temporizadorMacro = new ElapsedTime();
    private int pasoMacro = 0; 
    private int disparosRealizados = 0;

    private boolean ultimoTriangulo = false;
    private boolean ultimoCuadrado = false;
    private boolean ultimoCirculo = false;

    private int tagObjetivo = 1; 
    private HuskyLens.Block[] bloquesDetectados = new HuskyLens.Block[0];
    private HuskyLens.Block bloqueObjetivo = null;
    private double errorX = 0;

    @Override
    public void runOpMode() {
        InicializarHardware();

        telemetry.addData("🚨 VERSION", "V9 - CHÁSIS PERFECTO Y CONTROLES ORIGINALES");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                ActualizarVision();
                ControlarChasis();
                ControlarSistemasDeArmas();
                ActualizarTelemetriaPanel();
            }
        }
    }

    private void InicializarHardware() {
        Husky = hardwareMap.get(HuskyLens.class, "Husky");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        Index = hardwareMap.get(DcMotor.class, "Index");
        InTake = hardwareMap.get(DcMotor.class, "InTake");
        Torreta = hardwareMap.get(CRServo.class, "Torreta"); 
        Angulo = hardwareMap.get(CRServo.class, "Angulo"); 

        // CONFIGURACIÓN EXACTA DE LA BASE (V8)
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD); 
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        InTake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    private void ActualizarVision() {
        bloquesDetectados = Husky.blocks();
        bloqueObjetivo = null; 

        if (bloquesDetectados.length > 0) {
            for (HuskyLens.Block block : bloquesDetectados) {
                if (block.id == tagObjetivo) {
                    bloqueObjetivo = block;
                    errorX = bloqueObjetivo.x - 160; 
                    break; 
                }
            }
        }
    }

    private void ControlarChasis() {
        DefinirVelocidadChasis();
        CalcularCinematicaMecanum();
        VerificarApagadoMaestro();
    }

    private void DefinirVelocidadChasis() {
        if (gamepad1.triangle) MultiplicadorVelocidad = 0.6; 
        if (gamepad1.square) MultiplicadorVelocidad = 0.3;   
        if (gamepad1.cross) MultiplicadorVelocidad = 1.0;    
    }

    private void CalcularCinematicaMecanum() {
        double y = -gamepad1.left_stick_y;  // Adelante / Atrás
        double x = gamepad1.left_stick_x;   // Derecha / Izquierda
        double rx = gamepad1.right_stick_x; // Giro

        // MATRIZ V8 CALIBRADA
        double PowerFL = (y + x + rx) * MultiplicadorVelocidad;
        double PowerBL = (-y + x - rx) * MultiplicadorVelocidad;
        double PowerFR = (y - x - rx) * MultiplicadorVelocidad;
        double PowerBR = (-y - x + rx) * MultiplicadorVelocidad;

        FrontLeft.setPower(Math.min(Math.max(PowerFL, -1.0), 1.0));
        BackLeft.setPower(Math.min(Math.max(PowerBL, -1.0), 1.0));
        FrontRight.setPower(Math.min(Math.max(PowerFR, -1.0), 1.0));
        BackRight.setPower(Math.min(Math.max(PowerBR, -1.0), 1.0));
    }

    private void VerificarApagadoMaestro() {
        if (gamepad1.circle) { 
            macroDisparoActivo = false;
            autoShooterActivo = false;
            trackingTorretaActivo = false;
            Shooter.setPower(0);
            Index.setPower(0);
            InTake.setPower(0);
            Torreta.setPower(0);
        }
    }

    private void ControlarSistemasDeArmas() {
        GestionarTorreta();
        GestionarAngulo();
        GestionarMacroRafaga();
        
        if (!macroDisparoActivo) {
            GestionarIntake1();
            GestionarIntake2();
            GestionarShooter();
        }
        
        ActualizarEstadoDeBotones();
    }

    private void GestionarMacroRafaga() {
        if (gamepad2.circle && !ultimoCirculo) { 
            macroDisparoActivo = !macroDisparoActivo;
            if (macroDisparoActivo) {
                disparosRealizados = 0;
                pasoMacro = 1; 
                temporizadorMacro.reset();
            }
        }

        // Cancelar si se mueve cualquier control manual
        if (macroDisparoActivo && (Math.abs(gamepad2.left_stick_y) > 0.1 || Math.abs(gamepad2.right_stick_y) > 0.1 || gamepad2.right_trigger > 0.05 || gamepad2.left_trigger > 0.05 || gamepad2.right_bumper || gamepad2.left_bumper)) {
            macroDisparoActivo = false;
        }

        if (macroDisparoActivo) {
            switch (pasoMacro) {
                case 1:
                    Shooter.setPower(-1.0); 
                    InTake.setPower(1.0);   
                    Index.setPower(0);      
                    if (temporizadorMacro.milliseconds() >= 1500) { 
                        pasoMacro = 2; 
                        temporizadorMacro.reset();
                    }
                    break;
                case 2:
                    Shooter.setPower(-1.0);
                    InTake.setPower(1.0);   
                    Index.setPower(-0.6);   
                    if (temporizadorMacro.milliseconds() >= 190) { 
                        disparosRealizados++;
                        if (disparosRealizados >= 3) { 
                            macroDisparoActivo = false;
                            Shooter.setPower(0);
                            Index.setPower(0);
                            InTake.setPower(0);
                        } else {
                            pasoMacro = 3; 
                            temporizadorMacro.reset();
                        }
                    }
                    break;
                case 3:
                    Shooter.setPower(-1.0);
                    InTake.setPower(1.0);
                    Index.setPower(0.6);
                    if (temporizadorMacro.milliseconds() >= 200) {
                        pasoMacro = 2;
                        temporizadorMacro.reset();
                    }
                    break;
            }
        }
    }

    private void GestionarIntake1() {
        // INTAKE 1: Joystick Derecho (Arriba IN, Abajo OUT)
        if (gamepad2.right_stick_y < -0.1) {
            InTake.setPower(1.0); 
        } else if (gamepad2.right_stick_y > 0.1) {
            InTake.setPower(-1.0); 
        } else {
            InTake.setPower(0);
        }
    }

    private void GestionarIntake2() {
        // INTAKE 2 (Index): Joystick Izquierdo (Arriba IN, Abajo OUT)
        if (Math.abs(gamepad2.left_stick_y) > 0.1) {
            // El valor de Y es negativo hacia arriba, esto lo mapea directamente
            Index.setPower(gamepad2.left_stick_y); 
        } else {
            Index.setPower(0);
        }
    }

    private void GestionarShooter() {
        if (gamepad2.square && !ultimoCuadrado) { 
            autoShooterActivo = !autoShooterActivo;
        }
        
        // SHOOTER: Triggers (Derecho Disparar/IN, Izquierdo Reversa/OUT)
        if (gamepad2.right_trigger > 0.05) {
            autoShooterActivo = false;
            Shooter.setPower(-gamepad2.right_trigger); 
        } else if (gamepad2.left_trigger > 0.05) {
            autoShooterActivo = false;
            Shooter.setPower(gamepad2.left_trigger); 
        } else if (autoShooterActivo) { 
            Shooter.setPower(-1.0); 
        } else {
            Shooter.setPower(0);
        }
    }

    private void GestionarTorreta() {
        if (gamepad2.triangle && !ultimoTriangulo) { 
            trackingTorretaActivo = !trackingTorretaActivo;
        }
        
        // TORRETA: Bumpers (Control Manual)
        if (gamepad2.right_bumper) {
            trackingTorretaActivo = false;
            Torreta.setPower(1.0);
        } else if (gamepad2.left_bumper) {
            trackingTorretaActivo = false;
            Torreta.setPower(-1.0);
        } else if (trackingTorretaActivo && bloqueObjetivo != null) { 
            double Kp = 0.015;
            double potenciaTorreta = errorX * Kp;
            potenciaTorreta = Math.max(-0.5, Math.min(0.5, potenciaTorreta));
            
            if (Math.abs(errorX) > 15) {
                Torreta.setPower(potenciaTorreta);
            } else {
                Torreta.setPower(0);
            }
        } else {
            Torreta.setPower(0);
        }
    }

    private void GestionarAngulo() {
        if (gamepad2.dpad_up) { 
            Angulo.setPower(-0.5);
        } else if (gamepad2.dpad_down) { 
            Angulo.setPower(0.5);
        } else {
            Angulo.setPower(0);
        }
    }

    private void ActualizarEstadoDeBotones() {
        ultimoTriangulo = gamepad2.triangle;
        ultimoCuadrado = gamepad2.square;
        ultimoCirculo = gamepad2.circle;
    }

    private void ActualizarTelemetriaPanel() {
        telemetry.addData("CHASIS", "===================");
        telemetry.addData("MULTIPLICADOR", "%.1fx", MultiplicadorVelocidad);
        
        telemetry.addLine();
        telemetry.addData("ARMAS E INTAKES", "===================");
        telemetry.addData("INTAKE 1 (R-Stick)", "Pwr: %.2f", InTake.getPower());
        telemetry.addData("INTAKE 2 (L-Stick)", "Pwr: %.2f", Index.getPower());
        telemetry.addData("SHOOTER (Triggers)", "Pwr: %.2f", Shooter.getPower());
        telemetry.addData("TORRETA (Bumpers)", "Pwr: %.2f", Torreta.getPower());
        
        telemetry.update(); 
    }
}