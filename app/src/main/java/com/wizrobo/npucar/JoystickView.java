
package com.wizrobo.npucar;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PixelFormat;
import android.graphics.Point;
import android.graphics.PorterDuff;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;

//import android.graphics.AvoidXfermode;


public class JoystickView extends SurfaceView implements SurfaceHolder.Callback, View.OnTouchListener
{
    private SurfaceHolder mHolder;
    private Paint mPaint;
    public Point mRockerPosition; // 摇杆位置
    private Point mCtrlPoint;// 摇杆起始位置
    private int mRudderRadius = 25;// 摇杆半径
    private int mWheelRadius = 80;// 摇杆活动范围半径
    private int batmapHW = 160;
    private int batmap2HW = 40;


    int isHide = 0;
    Bitmap bitmap,bitmap2;
    float scale;
    private JoystickListener joystickCallback;
    public JoystickView(Context context)
    {
        super(context);
        getHolder().addCallback(this);
        setOnTouchListener(this);
        if(context instanceof JoystickListener)
            joystickCallback = (JoystickListener) context;
    }

    public JoystickView(Context context, AttributeSet attributes, int style)
    {
        super(context, attributes, style);
        getHolder().addCallback(this);
        setOnTouchListener(this);
        if(context instanceof JoystickListener)
            joystickCallback = (JoystickListener) context;
    }

    public JoystickView (Context context, AttributeSet attributes)
    {
        super(context, attributes);
        getHolder().addCallback(this);
        setOnTouchListener(this);
        if(context instanceof JoystickListener)
            joystickCallback = (JoystickListener) context;

        this.setKeepScreenOn(true);
        scale = (context.getResources().getDisplayMetrics().density)*0.8f;
        getHolder().addCallback(this);
        //setOnTouchListener(this);

        mRudderRadius = dip2px(15);// 摇杆半径
        mWheelRadius = dip2px(45);// 摇杆活动范围半径
        mCtrlPoint = new Point((mRudderRadius + mWheelRadius), (mRudderRadius + mWheelRadius));// 摇杆起始位置
        batmapHW = (mWheelRadius+mRudderRadius) * 2;
        batmap2HW = mRudderRadius * 2;
        mHolder = getHolder();
        mHolder.addCallback(this);
        mPaint = new Paint();
        mPaint.setAntiAlias(true);
        setFocusable(true);
        setFocusableInTouchMode(true);
        mRockerPosition = new Point(mCtrlPoint);
        setZOrderOnTop(true);
        mHolder.setFormat(PixelFormat.TRANSPARENT);//设置背景透明
        bitmap = BitmapFactory.decodeResource(getResources(), R.drawable.joystick_l_pad);
        bitmap = Bitmap.createScaledBitmap(bitmap, batmapHW, batmapHW, false);
        bitmap2 = BitmapFactory.decodeResource(getResources(), R.drawable.print2);
        bitmap2 = Bitmap.createScaledBitmap(bitmap2,batmap2HW,batmap2HW, false);
    }


    private void drawJoystick()
    {
        Canvas canvas = null;
        try {
            if (isHide == 0) {
                canvas = mHolder.lockCanvas();
                canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);//清除屏幕
                canvas.drawBitmap(bitmap, mCtrlPoint.x - mWheelRadius - mRudderRadius, mCtrlPoint.y - mWheelRadius - mRudderRadius, mPaint);
                canvas.drawBitmap(bitmap2, mRockerPosition.x - mRudderRadius, mRockerPosition.y - mRudderRadius, mPaint);
            }else {
                canvas = mHolder.lockCanvas();
                canvas.drawColor(Color.TRANSPARENT, PorterDuff.Mode.CLEAR);//清除屏幕
            }
        } catch (Exception e) {
            e.printStackTrace();
        } finally {
            if(canvas != null) {
                mHolder.unlockCanvasAndPost(canvas);
            }
        }
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder)
    {
        drawJoystick();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {

    }
    int len;
    @Override
    public boolean onTouch(View v, MotionEvent event)
    {

        try {
            if (isHide == 0) {
                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        len = MathUtils.getLength(mCtrlPoint.x, mCtrlPoint.y, event.getX(), event.getY());
                        //如果屏幕接触点不在摇杆挥动范围内,则不处理
                        if(len > mWheelRadius) {
                            return true;
                        }
                        break;
                    case MotionEvent.ACTION_MOVE:
                        len = MathUtils.getLength(mCtrlPoint.x, mCtrlPoint.y, event.getX(), event.getY());
                        if(len <= mWheelRadius) {
                            //如果手指在摇杆活动范围内，则摇杆处于手指触摸位置
                            mRockerPosition.set((int)event.getX(), (int)event.getY());

                        }else{
                            //设置摇杆位置，使其处于手指触摸方向的 摇杆活动范围边缘
                            mRockerPosition = MathUtils.getBorderPoint(mCtrlPoint, new Point((int)event.getX(), (int)event.getY()), mWheelRadius);
                        }

                        float radian = MathUtils.getRadian(mCtrlPoint, new Point((int)event.getX(), (int)event.getY()));
                        joystickCallback.onJoystickMoved((float)Math.sin(Math.PI*getAngleCouvert(radian)/180),(float)Math.cos(Math.PI*getAngleCouvert(radian)/180),getId());
                        break;
                    case MotionEvent.ACTION_UP:
                        mRockerPosition = new Point(mCtrlPoint);
                        joystickCallback.onJoystickMoved(0,0,getId());
                        break;
                }
                drawJoystick();
                Thread.sleep(60);
            }else {
                Thread.sleep(200);
            }
        } catch (Exception e) {

        }
        return true;
    }


    public int dip2px(float dpValue) {
        return (int)(dpValue * scale + 0.5f);
    }

    private int getAngleCouvert(float radian) {
        int tmp = (int)Math.round(radian/Math.PI * 180);
        if(tmp < 0) {
            return -tmp;
        }else{
            return 180 + (180 - tmp);
        }
    }


    public interface JoystickListener
    {
        void onJoystickMoved(float xPercent, float yPercent, int id);
    }

}

