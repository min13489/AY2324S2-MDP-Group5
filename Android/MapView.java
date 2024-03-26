package com.example.mdptesttest;

import android.content.Context;
import android.content.DialogInterface;
import android.content.res.Resources;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.AttributeSet;
import android.util.Log;
import android.util.TypedValue;
import android.view.DragEvent;
import android.view.View;
import android.widget.Toast;

import androidx.appcompat.app.AlertDialog;
import androidx.core.content.ContextCompat;
import androidx.core.widget.TextViewCompat;

public class MapView extends View implements View.OnDragListener{
    private MainActivity mainActivity;
    private static int rows = 20;
    private static int cols = 20;
    private static int[][] gridValues; // 2D array to hold cell values
    private Paint gridPaint;
    private int cellWidth;
    private int cellHeight;
    private boolean firstDraw = true;
    public RobotView robot;
    public ObstacleView item1, item2, item3, item4, item5, item6, item7, item8;
    public ObstacleView[] obstacleViews = new ObstacleView[8];

    public MapView(Context context) {
        super(context);
        init();
    }

    public MapView(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    private void init() {
        gridPaint = new Paint();
        gridPaint.setColor(Color.BLACK);
        gridPaint.setStyle(Paint.Style.STROKE);
        gridPaint.setStrokeWidth(2);
        gridValues = new int[rows][cols];

    }

    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        cellWidth = w / cols;
        cellHeight = h / rows;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (firstDraw) {
            setStartObstaclePos(item1);
            setStartObstaclePos(item2);
            setStartObstaclePos(item3);
            setStartObstaclePos(item4);
            setStartObstaclePos(item5);
            setStartObstaclePos(item6);
            setStartObstaclePos(item7);
            setStartObstaclePos(item8);
            firstDraw = false;
        }
        // Draw grid lines
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                int x = j * cellWidth;
                int y = i * cellHeight;

                if (gridValues[i][j] == 1) {
                    // Draw filled rectangle with black color if cell value is 1
                    Paint fillPaintBlack = new Paint();
                    fillPaintBlack.setColor(Color.BLACK);
                    canvas.drawRect(x, y, x + cellWidth, y + cellHeight, fillPaintBlack);
                } else if (gridValues[i][j] == 2) {
                    // Draw filled rectangle with green color if cell value is 2
                    Paint fillPaintGreen = new Paint();
                    fillPaintGreen.setColor(Color.GREEN);
                    canvas.drawRect(x, y, x + cellWidth, y + cellHeight, fillPaintGreen);
                } else {
                    // Draw empty rectangle if cell value is 0
                    canvas.drawRect(x, y, x + cellWidth, y + cellHeight, gridPaint);
                    if(i == rows -1){
                        String text = String.valueOf(j);
                        Paint textPaint = new Paint();
                        textPaint.setColor(Color.GRAY);
                        textPaint.setTextSize(16);
                        if (j<10) {
                            canvas.drawText(text, x + cellWidth / 2f - 6, y + cellHeight / 2f + 4, textPaint);
                        }
                        else{
                            canvas.drawText(text, x + cellWidth / 2f - 9, y + cellHeight / 2f + 4, textPaint);
                        }
                    }
                    if(i != rows -1 && j == 0){
                        String text = String.valueOf(19 - i);
                        Paint textPaint = new Paint();
                        textPaint.setColor(Color.GRAY);
                        textPaint.setTextSize(16);
                        if (j<10) {
                            canvas.drawText(text, x + cellWidth / 2f - 9, y + cellHeight / 2f, textPaint);
                        }
                        else {
                            canvas.drawText(text, x + cellWidth / 2f + 4, y + cellHeight / 2f + 10, textPaint);
                        }
                    }
                }
            }
        }

    }

    public void setGridPaint(int color) {
        gridPaint.setColor(color);
    }

    public void moveRobotUp() {
        if (robot.currentCenterRow >= 2) {
            robot.currentCenterRow -= 1;
            robot.currentFrontRow -= 1;
            robot.setY(robot.currentCenterRow * cellHeight + 5); //for some reason its misaligned so added offset
        }
        else {
            Toast.makeText(getContext(), "Cannot move further forward!", Toast.LENGTH_SHORT).show();
        }
    }
    public void moveRobotDown() {
        if (robot.currentCenterRow <= 17) {
            robot.currentCenterRow += 1;
            robot.currentFrontRow += 1;
            robot.setY(robot.currentCenterRow * cellHeight + 5); //for some reason its misaligned so added offset
        }
        else {
            Toast.makeText(getContext(), "Cannot move further backward!", Toast.LENGTH_SHORT).show();
        }
    }

    public void moveRobotRight() {
        if (robot.currentCenterCol <= 17) {
            robot.currentCenterCol += 1;
            robot.currentFrontCol += 1;
            robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
        }
        else {
            Toast.makeText(getContext(), "Cannot move further right!", Toast.LENGTH_SHORT).show();
        }
    }

    public void moveRobotLeft() {
        if (robot.currentCenterCol >= 2) {
            robot.currentCenterCol -= 1;
            robot.currentFrontCol -= 1;
            robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
        }
        else {
            Toast.makeText(getContext(), "Cannot move further left!", Toast.LENGTH_SHORT).show();
        }
    }

    public void setRobotPos(int row, int col, String dir) {
        robot.setX(col * cellWidth - 33);
        robot.setY(row * cellHeight + 25);
        if (dir.equals("N")) {
            robot.setN();
        } else if (dir.equals("S")) {
            robot.setS();
        } else if (dir.equals("W")) {
            robot.setW();
        } else if (dir.equals("E")) {
            robot.setE();
        }
    }
    public void rotateRobotURight () {
        if (robot.currentFrontRow < robot.currentCenterRow && robot.currentFrontCol == robot.currentCenterCol) { //N
            if (robot.currentCenterRow - 2 >= 1 && robot.currentCenterCol + 2 <= 18) {
                robot.currentCenterRow -= 2;
                robot.currentFrontRow -= 1;
                robot.currentCenterCol += 2;
                robot.currentFrontCol += 3;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate top right!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontRow > robot.currentCenterRow && robot.currentFrontCol == robot.currentCenterCol) { //S
            if (robot.currentCenterRow + 2 <= 18 && robot.currentCenterCol - 2 >= 1) {
                robot.currentCenterRow += 2;
                robot.currentFrontRow += 1;
                robot.currentCenterCol -= 2;
                robot.currentFrontCol -= 3;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate top right!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontCol > robot.currentCenterCol && robot.currentFrontRow == robot.currentCenterRow) { //E
            if (robot.currentCenterRow + 2 <= 18 && robot.currentCenterCol + 2 <= 18) {
                robot.currentCenterRow += 2;
                robot.currentFrontRow += 3;
                robot.currentCenterCol += 2;
                robot.currentFrontCol += 1;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate top right!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontCol < robot.currentCenterCol && robot.currentFrontRow == robot.currentCenterRow) { //W
            if (robot.currentCenterRow - 2 >= 1 && robot.currentCenterCol - 2 >= 1) {
                robot.currentCenterRow -= 2;
                robot.currentFrontRow -= 3;
                robot.currentCenterCol -= 2;
                robot.currentFrontCol -= 1;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate top right!", Toast.LENGTH_SHORT).show();
            }
        }

    }

    public void rotateRobotULeft () {
        if (robot.currentFrontRow < robot.currentCenterRow && robot.currentFrontCol == robot.currentCenterCol) { //N
            if (robot.currentCenterRow - 2 >= 1 && robot.currentCenterCol - 2 >= 1) {
                robot.currentCenterRow -= 2;
                robot.currentFrontRow -= 1;
                robot.currentCenterCol -= 2;
                robot.currentFrontCol -= 3;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateCounterClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate top left!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontRow > robot.currentCenterRow && robot.currentFrontCol == robot.currentCenterCol) { //S
            if (robot.currentCenterRow + 2 <= 18 && robot.currentCenterCol + 2 <= 18) {
                robot.currentCenterRow += 2;
                robot.currentFrontRow += 1;
                robot.currentCenterCol += 2;
                robot.currentFrontCol += 3;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateCounterClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate top left!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontCol > robot.currentCenterCol && robot.currentFrontRow == robot.currentCenterRow) { //E
            if (robot.currentCenterRow - 2 >= 1 && robot.currentCenterCol + 2 <= 18) {
                robot.currentCenterRow -= 2;
                robot.currentFrontRow -= 3;
                robot.currentCenterCol += 2;
                robot.currentFrontCol += 1;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateCounterClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate top left!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontCol < robot.currentCenterCol && robot.currentFrontRow == robot.currentCenterRow) { //W
            if (robot.currentCenterRow + 2 <= 18 && robot.currentCenterCol - 2 >= 1) {
                robot.currentCenterRow += 2;
                robot.currentFrontRow += 3;
                robot.currentCenterCol -= 2;
                robot.currentFrontCol -= 1;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateCounterClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate top left!", Toast.LENGTH_SHORT).show();
            }
        }
    }

    public void rotateRobotBLeft () {
        if (robot.currentFrontRow < robot.currentCenterRow && robot.currentFrontCol == robot.currentCenterCol) { //N
            if (robot.currentCenterRow + 2 <= 18 && robot.currentCenterCol - 2 >= 1) {
                robot.currentCenterRow += 2;
                robot.currentFrontRow += 3;
                robot.currentCenterCol -= 2;
                robot.currentFrontCol -= 1;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate bottom left!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontRow > robot.currentCenterRow && robot.currentFrontCol == robot.currentCenterCol) { //S
            if (robot.currentCenterRow - 2 >= 1 && robot.currentCenterCol + 2 <= 18) {
                robot.currentCenterRow -= 2;
                robot.currentFrontRow -= 3;
                robot.currentCenterCol += 2;
                robot.currentFrontCol += 1;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate bottom left!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontCol > robot.currentCenterCol && robot.currentFrontRow == robot.currentCenterRow) { //E
            if (robot.currentCenterRow - 2 >= 1 && robot.currentCenterCol - 2 >= 1) {
                robot.currentCenterRow -= 2;
                robot.currentFrontRow -= 1;
                robot.currentCenterCol -= 2;
                robot.currentFrontCol -= 3;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate bottom left!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontCol < robot.currentCenterCol && robot.currentFrontRow == robot.currentCenterRow) { //W
            if (robot.currentCenterRow + 2 <= 18 && robot.currentCenterCol + 2 <= 18) {
                robot.currentCenterRow += 2;
                robot.currentFrontRow += 1;
                robot.currentCenterCol += 2;
                robot.currentFrontCol += 3;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate bottom left!", Toast.LENGTH_SHORT).show();
            }
        }
    }

    public void rotateRobotBRight () {
        if (robot.currentFrontRow < robot.currentCenterRow && robot.currentFrontCol == robot.currentCenterCol) { //N
            if (robot.currentCenterRow + 3 <= 18 && robot.currentCenterCol + 2 <= 18) {
                robot.currentCenterRow += 2;
                robot.currentFrontRow += 3;
                robot.currentCenterCol += 2;
                robot.currentFrontCol += 1;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateCounterClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate bottom right!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontRow > robot.currentCenterRow && robot.currentFrontCol == robot.currentCenterCol) { //S
            if (robot.currentCenterRow - 2 >= 1 && robot.currentCenterCol - 2 >= 1) {
                robot.currentCenterRow -= 2;
                robot.currentFrontRow -= 3;
                robot.currentCenterCol -= 2;
                robot.currentFrontCol -= 1;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateCounterClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate bottom right!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontCol > robot.currentCenterCol && robot.currentFrontRow == robot.currentCenterRow) { //E
            if (robot.currentCenterRow + 2 <= 18 && robot.currentCenterCol - 2 >= 1) {
                robot.currentCenterRow += 2;
                robot.currentFrontRow += 1;
                robot.currentCenterCol -= 2;
                robot.currentFrontCol -= 3;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateCounterClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate bottom right!", Toast.LENGTH_SHORT).show();
            }
        }
        else if (robot.currentFrontCol < robot.currentCenterCol && robot.currentFrontRow == robot.currentCenterRow) { //W
            if (robot.currentCenterRow - 2 >= 1 && robot.currentCenterCol + 2 <= 18) {
                robot.currentCenterRow -= 2;
                robot.currentFrontRow -= 1;
                robot.currentCenterCol += 2;
                robot.currentFrontCol += 3;
                robot.setX(robot.currentCenterCol * cellHeight + 2); //for some reason its misaligned so added offset
                robot.setY(robot.currentCenterRow * cellHeight + 5);
                robot.rotateCounterClockwise();
                Log.d("Rotating", "fx: " + robot.currentFrontCol + " fy: " + robot.currentFrontRow + " cx: " + robot.currentCenterCol + " cy: " + robot.currentCenterRow);

            }
            else {
                Toast.makeText(getContext(), "Cannot move rotate bottom right!", Toast.LENGTH_SHORT).show();
            }
        }
    }

    public void setStartObstaclePos(ObstacleView obstacleView) {
        obstacleView.startX = obstacleView.getX();
        obstacleView.startY = obstacleView.getY();
    }

    private boolean isValidDropPos(int row, int col) {
        return row >= 0 && row < rows && col >= 0 && col < cols;
    }

    private boolean isSpaceAvail(int row, int col) {
        return gridValues[row][col] == 0;
    }

    public static void clearMap(int row, int col) {
                    gridValues[row][col] = 0;
                }

    public void clearAllMap() {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                gridValues[i][j] = 0;
            }
        }
    }

    /*
    @Override
    public boolean onTouchEvent(MotionEvent event) {
        int action = event.getAction();
        if (action == MotionEvent.ACTION_DOWN || action == MotionEvent.ACTION_MOVE) {
            int x = (int) event.getX();
            int y = (int) event.getY();
            int row = y / cellHeight;
            int col = x / cellWidth;
            if (row >= 0 && row < rows && col >= 0 && col < cols) {
                if (gridValues[row][col] == 1) {
                    gridValues[row][col] = 0; // Change cell value to 0 (white)
                } else {
                    Log.d("MAZE", "X: " + x + " Y: " + y);
                    Log.d("MAZE", "Col: " + col + " Row: " + row);
                    gridValues[row][col] = 1; // Change cell value to 1 (black)
                }
                invalidate(); // Redraw the view
                return true;
            }
        }
        return super.onTouchEvent(event);
    }
     */

    @Override
    public boolean onDrag(View v, DragEvent event) {
        switch (event.getAction()) {
            case DragEvent.ACTION_DRAG_ENDED:
                View resetView = (View) event.getLocalState();
                if (resetView instanceof ObstacleView) {
                    if (!(event.getX() >= 0 && event.getX() <= getWidth() && event.getY() >= 0 && event.getY() <= getHeight())) {
                        ObstacleView resetObstacle = (ObstacleView) resetView;
                        clearMap(resetObstacle.row, resetObstacle.col);
                        resetObstacle.clearObstacleData();
                        Log.d("Dropped Obstacle", resetObstacle.obstacleData);
                        invalidate();
                        resetObstacle.setStartPos();
                        MainActivity.sendObstacle("CLEAR," + resetObstacle.obstacleId);
                        resetObstacle.setText("");
                        resetObstacle.setVisibility(View.VISIBLE);
                        invalidate();
                    }
                }
                break;
            case DragEvent.ACTION_DRAG_STARTED:
                View draggedView = (View) event.getLocalState();
                if (draggedView instanceof ObstacleView) {
                    ObstacleView draggedObstacle = (ObstacleView) draggedView;
                    Log.d("DRAGGING", "X: " + draggedObstacle.row + " Y: " +draggedObstacle.col);
                    clearMap(draggedObstacle.row, draggedObstacle.col);
                }
                break;
            case DragEvent.ACTION_DROP:
                View droppedView = (View) event.getLocalState();
                // Check if the drop event occurred within the bounds of the MapView
                if (event.getX() >= 0 && event.getX() <= getWidth() && event.getY() >= 0 && event.getY() <= getHeight()) {
                    if (droppedView instanceof ObstacleView) {
                        float x = event.getX();
                        float y = event.getY();

                        int row = (int) (y / cellHeight);
                        int col = (int) (x / cellWidth);

                        // Check if the drop position is valid and there is enough space
                        if (isValidDropPos(row, col) && isSpaceAvail(row, col)) {
                            ((ObstacleView) droppedView).row = row;
                            ((ObstacleView) droppedView).col = col;
                            /*
                            for (int i = row; i < row + 2; i++) {
                                for (int j = col; j < col + 2; j++) {
                                    if (i >= 0 && i < rows && j >= 0 && j < cols) {
                                        gridValues[i][j] = 1;
                                    }
                                }
                               }
                             */
                            gridValues[row][col] = 1;
                            // Place obstacle at dropped location
                            droppedView.setX(col * cellWidth + 8); //for some reason its misaligned so added offset
                            droppedView.setY(row * cellHeight + 66); //for some reason its misaligned so added offset
                            // droppedView.setX(col * cellWidth + 30); //for some reason its misaligned so added offset
                            //droppedView.setY(row * cellHeight + 36); //for some reason its misaligned so added offset
                            showOptionsDialog(col, row, (ObstacleView) droppedView);
                            droppedView.setVisibility(View.VISIBLE);
                            droppedView.requestLayout();
                            droppedView.invalidate();
                        } else {
                            Log.d("isValidDropPos", String.valueOf(isValidDropPos(row, col)));
                            Toast.makeText(getContext(), "Invalid drop position or not enough space!", Toast.LENGTH_SHORT).show();
                            droppedView.setVisibility(View.VISIBLE);
                        }
                    }
                }
                invalidate();
                break;
        }
        return true;
    }

    private void showOptionsDialog(int col, int row, ObstacleView droppedView) {
        Log.d("Option Selector", "Left - Row: " + row + ", Column: " + col);
        AlertDialog.Builder builder = new AlertDialog.Builder(getContext());
        builder.setTitle("Where is the image facing")
                .setItems(new CharSequence[]{"West", "East", "North", "South"}, new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialog, int which) {
                        // Handle the selected option
                        switch (which) {
                            case 0:
                                gridValues[row][col] = 2;
                                droppedView.setX(col * cellWidth + 9); //for some reason its misaligned so added offset
                                //droppedView.setX(col * cellWidth + 33);
                                droppedView.setObstacleData("W", row, col);
                                Log.d("Dropped Obstacle", droppedView.obstacleData);
                                invalidate();
                                MainActivity.sendObstacle(droppedView.obstacleData);
                                break;
                            case 1:
                                gridValues[row][col] = 2;
                                droppedView.setX(col * cellWidth + 4); //for some reason its misaligned so added offset
                                //droppedView.setX(col * cellWidth + 28);
                                droppedView.setObstacleData("E", row, col);
                                Log.d("Dropped Obstacle", droppedView.obstacleData);
                                invalidate();
                                MainActivity.sendObstacle(droppedView.obstacleData);
                                break;
                            case 2:
                                gridValues[row][col] = 2;
                                droppedView.setX(col * cellWidth + 6); //for some reason its misaligned so added offset
                                droppedView.setY(row * cellHeight + 69); //for some reason its misaligned so added offset
                                //droppedView.setY(row * cellHeight + 38);
                                droppedView.setObstacleData("N", row, col);
                                Log.d("Dropped Obstacle", droppedView.obstacleData);
                                invalidate();
                                MainActivity.sendObstacle(droppedView.obstacleData);
                                break;
                            case 3:
                                gridValues[row][col] = 2;
                                droppedView.setX(col * cellWidth + 6); //for some reason its misaligned so added offset
                                droppedView.setY(row * cellHeight + 63); //for some reason its misaligned so added offset
                                //droppedView.setY(row * cellHeight + 33);
                                droppedView.setObstacleData("S", row, col);
                                Log.d("Dropped Obstacle", droppedView.obstacleData);
                                invalidate();
                                MainActivity.sendObstacle(droppedView.obstacleData);
                                break;
                        }
                    }
                })
                .show();
    }

    public void setObsPos(ObstacleView Obs, int col, int row, String dir) {
        Obs.setX(col * cellWidth + 8);
        Obs.setY(row * cellHeight + 66);
        switch (dir) {
            case "W":
                gridValues[row][col] = 2;
                Obs.setX(col * cellWidth + 9);
                Obs.row = row;
                Obs.col = col;
                Obs.setObstacleData("W", row, col);
                Log.d("Dropped Obstacle", Obs.obstacleData);
                MainActivity.sendObstacle(Obs.obstacleData);
                invalidate();
                break;
            case "E":
                gridValues[row][col] = 2;
                Obs.setX(col * cellWidth + 4);
                Obs.row = row;
                Obs.col = col;
                Obs.setObstacleData("E", row, col);
                Log.d("Dropped Obstacle", Obs.obstacleData);
                MainActivity.sendObstacle(Obs.obstacleData);
                invalidate();
                break;
            case "N":
                gridValues[row][col] = 2;
                Obs.setX(col * cellWidth + 6);
                Obs.setY(row * cellHeight + 69);
                Obs.row = row;
                Obs.col = col;
                Obs.setObstacleData("N", row, col);
                Log.d("Dropped Obstacle", Obs.obstacleData);
                MainActivity.sendObstacle(Obs.obstacleData);
                invalidate();
                break;
            case "S":
                gridValues[row][col] = 2;
                Obs.setX(col * cellWidth + 6);
                Obs.setY(row * cellHeight + 63);
                Obs.row = row;
                Obs.col = col;
                Obs.setObstacleData("S", row, col);
                Log.d("Dropped Obstacle", Obs.obstacleData);
                MainActivity.sendObstacle(Obs.obstacleData);
                invalidate();
                break;


        }
    }

}


