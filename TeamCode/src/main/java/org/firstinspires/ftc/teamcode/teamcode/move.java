package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

//@TeleOp(name = "Wait for Red Color")
public class move extends LinearOpMode {
    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        // Инициализация колор-сканера
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        // Ждем, пока оператор не нажмет кнопку старт
        waitForStart();

        // Ждем, пока не обнаружим красный цвет
        while (opModeIsActive()) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.update();

            // Проверяем, превышает ли значение красного порог
            if (red > green && red > blue) {
                telemetry.addData("Status", "Detected Red Color!");
                telemetry.update();
                sleep(1000);
                 // Выход из цикла, если красный цвет обнаружен
            }

             // Небольшая задержка, чтобы не нагружать процессор
        }
    }
}
