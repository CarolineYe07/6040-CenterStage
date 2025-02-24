package org.firstinspires.ftc.teamcode.Auto;

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

public class PropVisionProcessor implements VisionProcessor {



    // camera can only see 2 positions at once
    // public Rect rectRight = new Rect(200, 240, 120, 120);
    public Rect rectLeft = new Rect(140, 120, 150, 150);
    public Rect rectMiddle = new Rect(450, 50, 150, 150);
    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    double satRectLeft = 0;
    double satRectMiddle = 0;

    double saturation = 60;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);

        if ((satRectMiddle > satRectLeft) && (satRectMiddle > saturation)) {
            return Selected.MIDDLE;
        } else if ((satRectLeft > satRectMiddle) && (satRectLeft > saturation)) {
            return Selected.LEFT;
        }
        return Selected.RIGHT;

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
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);


        selection = (Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                break;

            case MIDDLE:
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                break;

            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);

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

