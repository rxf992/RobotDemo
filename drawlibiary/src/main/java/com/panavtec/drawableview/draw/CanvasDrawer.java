package com.panavtec.drawableview.draw;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.RectF;

import com.panavtec.drawableview.DrawableViewConfig;


public class CanvasDrawer {

  private boolean showCanvasBounds;
  private Paint paint;
  private float scaleFactor = 1.0f;
  private RectF viewRect = new RectF();
  private RectF canvasRect = new RectF();

  public CanvasDrawer() {
    initPaint();
  }

  public void onDraw(Canvas canvas) {
    if (showCanvasBounds) {
      canvas.drawRect(canvasRect, paint);
    }
    canvas.translate(-viewRect.left, -viewRect.top);
    canvas.scale(scaleFactor, scaleFactor);
  }

  public void onScaleChange(float scaleFactor) {
    this.scaleFactor = scaleFactor;
  }

  public void onViewPortChange(RectF viewRect) {
    this.viewRect = viewRect;
  }

  public void onCanvasChanged(RectF canvasRect) {
    this.canvasRect = canvasRect;
  }

  private void initPaint() {
    paint = new Paint(Paint.ANTI_ALIAS_FLAG | Paint.DITHER_FLAG | Paint.FILTER_BITMAP_FLAG);
    paint.setStrokeWidth(2.0f);
    paint.setStyle(Paint.Style.STROKE);
  }

  public void setConfig(DrawableViewConfig config) {
    this.showCanvasBounds = config.isShowCanvasBounds();
  }
}
