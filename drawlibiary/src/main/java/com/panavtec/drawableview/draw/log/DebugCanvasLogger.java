package com.panavtec.drawableview.draw.log;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;

public class DebugCanvasLogger implements CanvasLogger {

  private Paint paint;

  public DebugCanvasLogger() {
    paint = new Paint(Paint.ANTI_ALIAS_FLAG | Paint.DITHER_FLAG | Paint.FILTER_BITMAP_FLAG);
    paint.setTextSize(25.0f);
  }

  public void log(Canvas canvas, RectF canvasRect, RectF viewRect, float scaleFactor) {
  }

  private static String toShortString(RectF rectf) {
    return "["
        + rectf.left
        + ','
        + rectf.top
        + "]["
        + rectf.right
        + ','
        + rectf.bottom
        + "] B["
        + rectf.width()
        + ","
        + rectf.height()
        + "]";
  }

  private void logLine(Canvas canvas, String text, int lineNumber) {
    canvas.drawText(text, 5, 30 * lineNumber, paint);
  }
}
