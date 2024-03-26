package com.example.mdptesttest;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.util.AttributeSet;
import android.util.TypedValue;
import android.view.View;

public class ObstacleView extends View {
    private static final int WIDTH = 1; // Width of the obstacle in cells
    private static final int HEIGHT = 1; // Height of the obstacle in cells
    private Paint obstaclePaint;
    public String obstacleId = "1";
    public String obstacleText = "";
    public Paint textPaint;
    public int row;
    public int col;
    public String obstacleData = "Not Placed";
    public float startX;
    public float startY;

    public ObstacleView(Context context) {
        super(context);
        init();
    }

    public ObstacleView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        obstaclePaint = new Paint();
        obstaclePaint.setColor(Color.BLACK); // Set the color of the obstacle
        obstaclePaint.setStyle(Paint.Style.FILL);

        textPaint = new Paint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTextSize(15);
        textPaint.setTextAlign(Paint.Align.CENTER);
    }

    public void setId(String text) {
        obstacleId = text;
        invalidate();
    }

    public void setObsPaint(int obsColor, int textColor) {
        obstaclePaint.setColor(obsColor);
        textPaint.setColor(textColor);
    }

    public void setText(String text) {
        obstacleText = text;
        if (text == "") textPaint.setTextSize(15);
        else textPaint.setTextSize(25);
        invalidate();
    }

    public void setStartPos(){
        setX(startX);
        setY(startY);
    }

    public void setObstacleData(String dir, int row, int col) {
        obstacleData = "OBS," + obstacleId + "," + row + "," + col + "," + dir;
    }

    public void clearObstacleData() {
        obstacleData = "Not Placed";
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        // Draw the obstacle as a filled rectangle
        canvas.drawRect(0, 0, getWidth(), getHeight(), obstaclePaint);
        int x = getWidth() / 2;
        int y = getHeight() / 2;
        Rect bounds = new Rect();
        if (obstacleText == "") {
            textPaint.getTextBounds(obstacleId, 0, obstacleId.length(), bounds);
            int textX = x - bounds.width() / 2 + 3;
            int textY = y + bounds.height() / 2;
            canvas.drawText(obstacleId, textX, textY, textPaint);}
        else {
            textPaint.getTextBounds(obstacleText, 0, obstacleText.length(), bounds);
            int textX;
            int textY;
            if (obstacleText.length() > 1)
            {
                textX = x - bounds.width() / 2 + 14;
                textY = y + bounds.height() / 2;
            }
            else {
                textX = x - bounds.width() / 2 + 3;
                textY = y + bounds.height() / 2;
            }
            canvas.drawText(obstacleText, textX, textY, textPaint);}
    }

}
