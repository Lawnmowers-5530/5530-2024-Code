package frc.robot.subsystems;

import org.opencv.core.Mat;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
    UsbCamera camera;
    Mat image = new Mat();
    CvSink sink;
    CvSource output;
    
    public Camera( String name, int port, int width, int height, int fps) {
        camera = CameraServer.startAutomaticCapture(name, port);
        camera.setVideoMode(PixelFormat.kMJPEG, width, height, fps);
        sink = CameraServer.getVideo(camera);
        output = CameraServer.putVideo(name, width, height);
    }

    @Override
    public void periodic() {
        if (!camera.isConnected()) {
            return;
        }
        long res = sink.grabFrame(image);
        if (res == 0) {
            return;
        }
        output.putFrame(image);
    }
}
