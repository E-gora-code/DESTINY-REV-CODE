package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Thread Limit Test", group = "Test")
public class ped extends LinearOpMode {

    private int successfulThreads = 0;
    private final Object lock = new Object();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        ElapsedTime timer = new ElapsedTime();

        // Попытка создания потоков до достижения предела
        try {
            for (int i = 0; i < 1000; i++) { // Пытаемся создать до 1000 потоков
                final int threadNumber = i;

                new Thread(new Runnable() {
                    @Override
                    public void run() {
                        synchronized (lock) {
                            successfulThreads++;
                        }
                        try {
                            // Поток работает 10 секунд
                            Thread.sleep(10000);
                        } catch (InterruptedException e) {
                            Thread.currentThread().interrupt();
                        }
                    }
                }, "TestThread-" + i).start();

                // Небольшая задержка между созданием потоков
                Thread.sleep(10);

                // Обновление телеметрии
                if (i % 10 == 0) {
                    telemetry.addData("Threads Created", i + 1);
                    telemetry.addData("Successful Threads", successfulThreads);
                    telemetry.addData("Time", timer.seconds());
                    telemetry.update();
                }
            }
        } catch (OutOfMemoryError e) {
            telemetry.addData("Error", "OutOfMemoryError at thread: " + successfulThreads);
        } catch (Throwable t) {
            telemetry.addData("Error", t.getClass().getSimpleName());
        }

        // Финальные результаты
        telemetry.addLine("=== FINAL RESULTS ===");
        telemetry.addData("Total Successful Threads", successfulThreads);
        telemetry.addData("Total Time", timer.seconds());
        telemetry.addData("Available Processors", Runtime.getRuntime().availableProcessors());
        telemetry.addData("Free Memory", Runtime.getRuntime().freeMemory() / 1024 / 1024 + " MB");
        telemetry.addData("Total Memory", Runtime.getRuntime().totalMemory() / 1024 / 1024 + " MB");
        telemetry.addData("Max Memory", Runtime.getRuntime().maxMemory() / 1024 / 1024 + " MB");

        telemetry.update();

        // Ожидание завершения работы
        while (opModeIsActive()) {
            sleep(1000);
        }
    }
}