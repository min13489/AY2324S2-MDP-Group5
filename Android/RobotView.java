package com.example.mdptesttest;

import android.content.ClipData;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.ColorFilter;
import android.graphics.ColorMatrix;
import android.graphics.Paint;
import android.graphics.PorterDuff;
import android.graphics.PorterDuffColorFilter;
import android.graphics.Rect;
import android.os.Build;
import android.util.AttributeSet;
import android.view.View;

public class RobotView extends View {
    private static final int WIDTH = 3; // Width of the obstacle in cells
    private static final int HEIGHT = 3; // Height of the obstacle in cells
    private Paint carPaint;
    private int angle = 0;
    public int startFrontRow = 17;
    public int startFrontCol = 1;
    public int startCenterCol = 1;
    public int currentFrontRow = 17;
    public int currentFrontCol= 1;
    public int currentCenterRow = 18;
    public int currentCenterCol = 1;
    public String dir = "N";

    public RobotView(Context context) {
        super(context);
        init();
    }

    public RobotView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        carPaint = new Paint();
        carPaint.setColor(Color.GRAY); // Set the color of the obstacle
        carPaint.setStyle(Paint.Style.FILL);
    }

    public void setN() {
        this.angle = 0;
        invalidate();
    }

    public void setS() {
        this.angle = 180;
        invalidate();
    }

    public void setE() {
        this.angle = 90;
        invalidate();
    }

    public void setW() {
        this.angle = 270;
        invalidate();
    }
    public void rotateClockwise() {
        angle += 90;
        invalidate();
    }

    public void rotateCounterClockwise() {
        angle -= 90;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        canvas.drawRect(0, 0, getWidth(), getHeight(), carPaint);
        Bitmap image = BitmapFactory.decodeResource(getResources(), R.drawable.car);
        float scaleX = (float) getWidth() / image.getWidth();
        float scaleY = (float) getHeight() / image.getHeight();
        float scale = Math.min(scaleX, scaleY);
        int newWidth = Math.round(image.getWidth() * scale);
        int newHeight = Math.round(image.getHeight() * scale);
        int centerX = (getWidth() - newWidth) / 2;
        int centerY = (getHeight() - newHeight) / 2;
        Bitmap scaledImage = Bitmap.createScaledBitmap(image, newWidth, newHeight, true);
        canvas.rotate(angle, getWidth() / 2f, getHeight() / 2f);
        canvas.drawBitmap(scaledImage, centerX, centerY, null);
    }

}
