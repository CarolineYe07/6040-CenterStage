package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.Console;

public class PropVisionProcessor implements VisionProcessor {



    // camera can only see 2 positions at once
    // public Rect rectRight = new Rect(200, 240, 120, 120);
    public Rect rectMiddle = new Rect(30, 30, 150, 150);
    public Rect rectRight = new Rect(390, 90, 150, 150);
    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    double satRectMiddle = 0;
    double satRectRight = 0;

    double saturation = 60;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        satRectRight = getAvgSaturation(hsvMat, rectRight);

        if ((satRectRight > satRectMiddle) && (satRectRight > saturation)) {
            return Selected.RIGHT;
        } else if ((satRectMiddle > satRectRight) && (satRectMiddle > saturation)) {
            return Selected.MIDDLE;
        }
        return Selected.LEFT;

    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth((scaleCanvasDensity * 4));

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        // 1.67
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);


        selection = (Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleRight, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                break;

            case MIDDLE:
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                break;

        }
    }

    public Selected getSelection() {
        return selection;
    }


    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }
}

