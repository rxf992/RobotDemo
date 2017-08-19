package com.wizrobo.npucar;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.graphics.Bitmap;
import android.graphics.Bitmap.Config;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Matrix;
import android.graphics.Paint;
import android.graphics.PaintFlagsDrawFilter;
import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.Rect;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.StrictMode;
import android.support.v7.app.AppCompatActivity;
import android.util.DisplayMetrics;
import android.util.Log;
import android.view.Gravity;
import android.view.KeyEvent;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.ViewGroup;
import android.view.ViewTreeObserver;
import android.view.ViewTreeObserver.OnGlobalLayoutListener;
import android.widget.AdapterView;
import android.widget.AdapterView.OnItemSelectedListener;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.ExpandableListView;
import android.widget.ExpandableListView.OnChildClickListener;
import android.widget.ExpandableListView.OnGroupCollapseListener;
import android.widget.ExpandableListView.OnGroupExpandListener;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.ScrollView;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import com.wizrobonpu.NpuIceI;
import com.wizrobonpu.WizRoboNpuUdp;

import java.io.IOException;
import java.io.OutputStream;
import java.lang.ref.WeakReference;
import java.net.InetAddress;
import java.net.Socket;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import Ice.Util;
import wizrobo_npu.ActionState;
import wizrobo_npu.CellMat;
import wizrobo_npu.CellType;
import wizrobo_npu.CoreParam;
import wizrobo_npu.ImgLidarScan;
import wizrobo_npu.ImgMap;
import wizrobo_npu.ImgPath;
import wizrobo_npu.ImgPoint;
import wizrobo_npu.ImgPose;
import wizrobo_npu.ImgStation;
import wizrobo_npu.LidarScan;
import wizrobo_npu.Map2D;
import wizrobo_npu.MapInfo;
import wizrobo_npu.MotorSpd;
import wizrobo_npu.NaviMode;
import wizrobo_npu.NaviState;
import wizrobo_npu.PathInfo;
import wizrobo_npu.PixelMat;
import wizrobo_npu.Point3D;
import wizrobo_npu.Pose3D;
import wizrobo_npu.ServerState;
import wizrobo_npu.SlamMode;
import wizrobo_npu.Station;
import wizrobo_npu.StationInfo;
import wizrobo_npu.StationType;
import wizrobo_npu.Vel3D;


public class WizRoboNpu extends AppCompatActivity implements JoystickView.JoystickListener {
    private Button bt_reboot;
    private Bitmap bm_background;
    private ImageView iv_background;
    private Spinner sp_map_list, sp_path_station_list;
    private TextView tv_map_info, tv_act_vel, tv_cmd_vel, tv_act_posex, tv_robot_status, tv_act_motor_spd, tv_cmd_motor_spd, tv_ip, tv_wifi_strength;
    private CheckBox cb_lidar_display, cb_path_display, cb_station_display;
    private ScrollView sv_control;
    private ImageButton ibt_delete_map, ibt_pause, ibt_cancel;
    private ExpandableListView elv_operate = null;
    private MyExpandableListAdapterOperate adapter_operate = null;
    private ExpandableListView elv_setting = null;
    private MyExpandableListAdapterSetting adapter_setting = null;
    private ExpandableListView elv_function = null;
    private MyExpandableListAdapterFunction adapter_function = null;

    ExpandableListView tempparent;
    View tempview;
    public static ViewGroup track_navi_parent;
    public static View track_navi_view;
    public static ViewGroup slam_parent;
    public static View slam_view;

    public static String strSavePath = "自动添加路径";
    public static String strModifyPathXY = "修改XY值";
    public static String strModifyPathYaw = "修改角度";
    public static String strSetInitialPose = "设置初始点";
    public static String strSetGoalPose = "设置自由路径";
    public static String strNaviTrack = "开始导航";
    public static String strSlam = "开始建图";

    public HashMap<String, List<String>> operateItemData = ExpandableData.getOperateData();
    public List<String> operateTitle = new ArrayList<String>(operateItemData.keySet());
    public HashMap<String, List<String>> settingItemData = ExpandableData.getSettingData();
    public List<String> settingTitle = new ArrayList<String>(settingItemData.keySet());
    public HashMap<String, List<String>> functionItemData = ExpandableData.getFuctionData();
    public List<String> functionTitle = new ArrayList<String>(functionItemData.keySet());

    ///*****Handler Class******
    Handler handler_get_map2d = new Handler();
    Handler handler_display_points = new Handler();
    Handler handler_update_ui = new Handler();

    private Thread manualThread;

    private final static int MESSAGECODE = 1;
    private static Handler imgmapHandler;
    private static boolean toGetImgMap = false;

    ///*****Communication Class*****
    public static String str_hostIP = new String();
    static NpuIceI mynpu = new NpuIceI();
    WizRoboNpuUdp udpSendIp = new WizRoboNpuUdp();
    private SlamMode slamMode = SlamMode.ICP_SLAM;

    ///*****Map Class*****
    static ImgMap imgMap = new ImgMap();
    MapInfo mapInfo = new MapInfo();
    MapInfo[] mapInfoList = null;
    Station[] stationList;
    ImgPath[] imgPathList = null;
    ImgStation[] imgStationList = null;
    wizrobo_npu.Path[] pathList;

    PixelMat pixelMat = new PixelMat();
    CellMat cellMat = new CellMat();
    Map2D map2d = new Map2D();
    Pose3D[] pathPose = new Pose3D[100];        //Limited
    Pose3D[] stationPathPose = new Pose3D[100]; //Limited
    ImgPose[] imgPathPose = new ImgPose[100];   //Limited
    ImgPoint[] imgPathPoint = new ImgPoint[100];//Limited
    wizrobo_npu.Path path2D = new wizrobo_npu.Path();
    static float resolution = 0.07f, redarrowYaw = 0;

    public static ImgPose actImgPose = new ImgPose(0, 0, 0);
    public static Pose3D actPose = new Pose3D(0, 0, 0, 0, 0, 0);
    public static Vel3D actVel = new Vel3D(0, 0, 0, 0, 0, 0);
    public static Vel3D cmdVel = new Vel3D(0, 0, 0, 0, 0, 0);
    public static MotorSpd cmdMotorSpd = new MotorSpd();
    public static MotorSpd actMotorSpd = new MotorSpd();
    public static NaviState naviState = NaviState.IDLE;
    public static wizrobo_npu.Path cmdPath = new wizrobo_npu.Path();
    public static ImgPath cmdImgPath = new ImgPath();
    public static LidarScan lidarScanData = new LidarScan();
    public static ImgLidarScan imglidarscandata = new ImgLidarScan();
    public static ImgPoint[] FootprintVerticles;


    ///*****Flag Class*****
        //是否执行自由路径
    boolean isSettingFreePath = false
        //是否执行一侧
            , displayOnce = false,
            //wifi是否连接
            wifiIsConnected = false,
            //是否正常运行中
            isRunning = true,
            //是否暂停
            pause = false,
            //改变子视图
            toChangeChildView = false,
            isPoseMode = false,
            isImgPoseMode = true,
            toRunOnce = true,
            toDisplayPath = true,
            toDisplayStation = true,
            actionMove = false,
            isReadThumbnail = false,
            isToDisplayInitPose = false,
            toStopGetImgMap = false,
            pointIsSelected = false;
    public static boolean isJoystickTriggered = false,
            //是否导航模式
            isTrack = false,
            //是否建图模式
            isSlam = false,
            isNavi = false,
            toMove = true,
            isSettingStationPath = false,
            updateUIOnce = true,
            toModifyPath = false,
            toModifyPathYaw = false;
    public static boolean isInited = false,
            toDisplayLidar = true,
            isTimeout = false,
            isSettingPathpose = false,
            isSettingInitialPose = false,
            isSettingGoalPose = false;
    //地图相关参数
    private static float mapEnlargeLevel = 1.0f, mapZoomScare = 1, mapZoomScareTemp = 1, canvasHeight, canvasWidth;
    private ArrayAdapter<String> adapterMapList, adapterPathStationList;
    private static List<String> listMapId, listPathStationId;
    public static String mapname = "0";
    private Matrix currentMatrix = new Matrix();
    private Matrix matrix = new Matrix();
    private PointF midPoint = new PointF();
    private PointF startPoint = new PointF();
    private MODE mode = MODE.NONE;
    private float startDis, mapZoomdx, mapZoomdy, mapDragdx, mapDragdy;
    private int pathPoseNum = 0, countTime = 0;

    private enum MODE {NONE, DRAG, ZOOM}

    ;
    private static int stationPathNum = 0, wifiStrength = 0;
    ///pose class
    private int actImgposeU, actImgposeV, initImgposeU, initImgposeV, setImgposeU, setImgposeV, robotInitPoseX, robotInitPoseY;
    private float actPoseX, actPoseY, actPoseYaw, actImgposeTheta, initImgposeTheta, setImgposeTheta;
    private float initPoseX, initPoseY, initPoseYaw, setPoseX, setPoseY, setPoseYaw;
    public static float linScale = 0, angScale = 0, poseYaw;

    Runnable update_display_points = new Runnable() {

        public void run() {
            ServerState serverState = mynpu.serverState;
            if (serverState == ServerState.TIMEOUT) {
                if (wifiStrength < -70) {
                    Utils.showToast(getApplicationContext(), "信号弱 " + mynpu.timeoutMethodName + "连接超时，正在重新连接！！！");
                }
                isTimeout = true;
                Utils.Delay(2000);
                ConnectNPU();
                isRunning = true;
            }

            if (isNavi || isSlam || isTrack) {
                try {
                    if (mynpu.isInited) {
                        Display();
                        if (toChangeChildView) {
                            toChangeChildView = false;
                            adapter_operate.getChildView(0, 1, true, tempview, tempparent);
                        }

                        if (isNavi || isSlam || isTrack) {
                            UpdateCmdVel();
                            UpdateActVel();
                            UpdateCmdMotorSpd();
                            UpdateActMotorSpd();
                            UpdateRobotStatus();
                            UpdateActPose();
                            GetWifiStrength();
                        }
                    }
                } catch (Exception e) {
                    ExceptionAlert(e);
                }
            }

            if (!mynpu.isInited && !displayOnce && wifiIsConnected) {
                countTime++;
                if (countTime == 8) {
                    countTime = 0;
                    displayOnce = true;
                    ShowDialog.showConnectFailed(WizRoboNpu.this);
                }
            }
            //每隔1s重新发送相同的请求
            handler_display_points.postDelayed(update_display_points, 1000);
        }
    };

    Runnable update_map2d = new Runnable() {
        public void run() {
            map2d = mynpu.GetCurrentMap();
            if (map2d == null)
                return;
            mapInfo = map2d.info;
            cellMat = map2d.mat;
            CellType[] data;
            data = cellMat.data;
            byte[] mapdata = new byte[data.length];
            if (cellMat.height == 0 || cellMat.width == 0)
                return;
            resolution = (float) mapInfo.resolution;

            tv_map_info.setText("MapInfo:" + Integer.toString(cellMat.width) + ", " + Integer.toString(cellMat.height) + ", " + Double.toString(mapInfo.resolution));

            if (cellMat.data.length == 0 || cellMat.height == 0 || cellMat.width == 0) {
                return;
            }
            for (int i = 0; i < data.length; i++) {
                if (data[i] == CellType.FREE_CELL)
                    mapdata[i] = (byte) 255;
                else if (data[i] == CellType.OCCUPIED_CELL)
                    mapdata[i] = (byte) 0;
                else
                    mapdata[i] = (byte) 127;
            }
            bm_background = GetBitmapFromPgm(mapdata, cellMat.width, cellMat.height, 0);
            bm_background = Convert(bm_background, cellMat.width, cellMat.height);

            handler_get_map2d.postDelayed(update_map2d, 2500);
        }
    };

    Runnable update_ui = new Runnable() {
        public void run() {
            try {
                if (mynpu.isInited) {
                    if (updateUIOnce) {
                        isInited = true;
                        tv_ip.setText(str_hostIP);
                        Utils.showToast(getApplicationContext(), "连接成功！");

                        CoreParam coreParam = mynpu.GetCoreParam();
                        mapname = coreParam.map_id;
                        ActionState actionState = mynpu.GetActionState();
                        NaviMode naviMode = mynpu.GetNaviMode();
                        if (actionState == ActionState.SLAM_ACTION) {
                            Slam();
                        }

                        if (actionState == ActionState.NAVI_ACTION) {

                            if (naviMode == NaviMode.P2P_NAVI) {
                                Navi();
                            } else if (naviMode == NaviMode.PF_NAVI) {
                                Track();
                            }
                        }

                        Log.d(TAG, "ActionState:" + actionState.name());
                        Log.d(TAG, "ActionStateCoreparam:" + coreParam.map_id);
                        updateUIOnce = false;
                        handler_update_ui.removeCallbacks(update_ui);
                    }
                }
            } catch (Exception e) {
                ExceptionAlert(e);

            }
            //0.1s 重新来刷新界面
            handler_update_ui.postDelayed(update_ui, 100);
        }
    };

    Handler handler = new Handler() {
        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            Bundle data = msg.getData();
            String val = data.getString("value");
            Log.d(TAG, "请求结果:" + val);
        }
    };

    Runnable runnable = new Runnable() {
        @Override
        public void run() {
            Message msg = new Message();
            Bundle data = new Bundle();
            data.putString("value", "请求结果");
            msg.setData(data);
            handler.sendMessage(msg);
        }
    };


    private static String TAG = "WizROboMpu";
    private Button btn_connection;
    private Button btn_disconnection;
    private Button btn_up;
    private Button btn_down;
    private Socket socket;
    private Button btn_down_1;
    private Button btn_up_1;
    private HashMap<String, String> dianjiMap;

    /**
     * Called when the activity is first created.
     */

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main_phone);
        new Thread(runnable).start();
        StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy);

        btn_connection = (Button) findViewById(R.id.btn_connection);
        btn_disconnection = (Button) findViewById(R.id.btn_disconnection);
        btn_up = (Button) findViewById(R.id.btn_up);
        btn_down = (Button) findViewById(R.id.btn_down);
        btn_down_1 = (Button) findViewById(R.id.btn_down_1);
        btn_up_1 = (Button) findViewById(R.id.btn_up_1);

        dianjiMap = new HashMap<>();
        btn_connection.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                toUpDianji();
                if (socket == null) {
                    try {
                        socket = new Socket("192.168.0.67", 8899);
                        if (socket.isConnected()) {
                            Utils.showToast(getApplicationContext(), "舵机已连接");
                            OutputStream outputStream = socket.getOutputStream();
                            outputStream.write(getCmd().getBytes());
                        }
                    } catch (Exception e) {
                        Utils.showToast(getApplicationContext(), "舵机连接出现问题");
                        e.printStackTrace();

                    }
                }
            }
        });
        btn_disconnection.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                toUpDianji();
                if (socket == null) {
                    Utils.showToast(getApplicationContext(), "连接已断开");
                    return;
                } else {
                    if (socket.isConnected()) {
                        try {
                            OutputStream outputStream = socket.getOutputStream();
                            outputStream.write(getCmd().getBytes());
                            socket.close();
                            socket = null;
                        } catch (Exception e) {
                            Utils.showToast(getApplicationContext(), "舵机连接出现问题");
                            e.printStackTrace();
                        }
                    } else {
                        Utils.showToast(getApplicationContext(), "连接已断开");
                    }

                }

            }
        });

        btn_down.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                toUpDianji();
                if (socket == null) {
                    try {
                        socket = new Socket("192.168.0.67", 8899);
                        if (socket.isConnected()) {
                            Utils.showToast(getApplicationContext(), "舵机已连接");
                            OutputStream outputStream = socket.getOutputStream();
                            outputStream.write(getCmd().getBytes());
                        }
                    } catch (Exception e) {
                        Utils.showToast(getApplicationContext(), "舵机连接出现问题");
                        e.printStackTrace();

                    }
                } else {
                    if (socket.isConnected()) {
                        try {
                            OutputStream outputStream = socket.getOutputStream();
                            outputStream.write(getCmd().getBytes());
                        } catch (Exception e) {
                            Utils.showToast(getApplicationContext(), "舵机连接出现问题");
                            e.printStackTrace();
                        }
                    }
                }

            }
        });

        btn_up.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                toDownDianji();
                if (socket == null) {
                    try {
                        socket = new Socket("192.168.0.67", 8899);
                        if (socket.isConnected()) {
                            Utils.showToast(getApplicationContext(), "舵机已连接");
                            OutputStream outputStream = socket.getOutputStream();
                            outputStream.write(getCmd().getBytes());
                        }
                    } catch (Exception e) {
                        Utils.showToast(getApplicationContext(), "舵机连接出现问题");
                        e.printStackTrace();

                    }
                } else {
                    if (socket.isConnected()) {
                        try {
                            OutputStream outputStream = socket.getOutputStream();
                            outputStream.write(getCmd().getBytes());
                        } catch (Exception e) {
                            Utils.showToast(getApplicationContext(), "舵机连接出现问题");
                            e.printStackTrace();
                        }
                    }
                }

            }
        });

        btn_up_1.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (dianjiMap.size() == 0) {
                    toUpDianji();
                }
                if (dianjiMap.containsKey("#1")) {
                    dianjiMap.put("#0", "P1200");
                }
                if (socket == null) {
                    try {
                        socket = new Socket("192.168.0.67", 8899);
                        if (socket.isConnected()) {
                            Utils.showToast(getApplicationContext(), "舵机已连接");
                            OutputStream outputStream = socket.getOutputStream();
                            outputStream.write(getCmd().getBytes());
                        }
                    } catch (Exception e) {
                        Utils.showToast(getApplicationContext(), "舵机连接出现问题");
                        e.printStackTrace();

                    }
                } else {
                    if (socket.isConnected()) {
                        try {
                            OutputStream outputStream = socket.getOutputStream();
                            outputStream.write(getCmd().getBytes());
                        } catch (Exception e) {
                            Utils.showToast(getApplicationContext(), "舵机连接出现问题");
                            e.printStackTrace();
                        }

                    }
                }

            }
        });

        btn_down_1.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                if (dianjiMap.size() == 0) {
                    toUpDianji();
                }
                if (dianjiMap.containsKey("#1")) {
                    dianjiMap.put("#0", "P1400");
                }
                if (socket == null) {
                    try {
                        socket = new Socket("192.168.0.67", 8899);
                        if (socket.isConnected()) {
                            Utils.showToast(getApplicationContext(), "舵机已连接");
                            OutputStream outputStream = socket.getOutputStream();
                            outputStream.write(getCmd().getBytes());
                        }
                    } catch (Exception e) {
                        Utils.showToast(getApplicationContext(), "舵机连接出现问题");
                        e.printStackTrace();

                    }
                } else {
                    if (socket.isConnected()) {
                        try {
                            OutputStream outputStream = socket.getOutputStream();
                            outputStream.write(getCmd().getBytes());
                        } catch (Exception e) {
                            Utils.showToast(getApplicationContext(), "舵机连接出现问题");
                            e.printStackTrace();
                        }
                    }
                }
            }
        });

        bt_reboot = (Button) findViewById(R.id.bt_reboot);
        bt_reboot.setVisibility(View.INVISIBLE);
        ibt_delete_map = (ImageButton) findViewById(R.id.ibt_delete_map);
        ibt_delete_map.setVisibility(View.INVISIBLE);
        ibt_pause = (ImageButton) findViewById(R.id.ibt_pause);
        ibt_pause.setVisibility(View.INVISIBLE);
        ibt_cancel = (ImageButton) findViewById(R.id.ibt_cancel);
        ibt_cancel.setVisibility(View.INVISIBLE);

        bt_reboot.setOnClickListener(Bt_Reboot_Onclick);
        ibt_delete_map.setOnClickListener(Ibt_DeleteMap_OnClick);
        ibt_pause.setOnClickListener(Ibt_Pause_OnClick);
        ibt_cancel.setOnClickListener(Ibt_Cancel_OnClick);

        cb_lidar_display = (CheckBox) findViewById(R.id.cb_lidar_display);
        cb_path_display = (CheckBox) findViewById(R.id.cb_path_display);
        cb_station_display = (CheckBox) findViewById(R.id.cb_station_display);
        cb_station_display.setVisibility(View.INVISIBLE);
        cb_lidar_display.setVisibility(View.INVISIBLE);
        cb_path_display.setVisibility(View.INVISIBLE);

        tv_ip = (TextView) findViewById(R.id.tv_ip);
        tv_map_info = (TextView) findViewById(R.id.tv_mapinfo);
        tv_act_posex = (TextView) findViewById(R.id.tv_act_pose);
        tv_act_vel = (TextView) findViewById(R.id.tv_act_vel);
        tv_cmd_vel = (TextView) findViewById(R.id.tv_cmd_vel);
        tv_act_motor_spd = (TextView) findViewById(R.id.tv_act_motor_spd);
        tv_cmd_motor_spd = (TextView) findViewById(R.id.tv_cmd_motor_spd);
        tv_robot_status = (TextView) findViewById(R.id.tv_robot_status);
        tv_wifi_strength = (TextView) findViewById(R.id.tv_wifi_strength);
        iv_background = (ImageView) findViewById(R.id.iv_background);
        sp_map_list = (Spinner) findViewById(R.id.Spinnermaplist);
        listMapId = new ArrayList<String>();
        adapterMapList = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, listMapId);
        sp_map_list.setAdapter(adapterMapList);
        listMapId.add("地图列表");
        adapterMapList.notifyDataSetChanged();

        sp_path_station_list = (Spinner) findViewById(R.id.sp_path_station);
        listPathStationId = new ArrayList<String>();
        adapterPathStationList = new ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, listPathStationId);
        sp_path_station_list.setAdapter(adapterPathStationList);

        sv_control = (ScrollView) findViewById(R.id.sv_control);
        sv_control.setVisibility(View.INVISIBLE);

        DisplayMetrics dm = new DisplayMetrics();
        getWindowManager().getDefaultDisplay().getMetrics(dm);

        //创建Handler
        imgmapHandler = new MyHandler(this);
        //创建线程并且启动线程
        new Thread(new MyRunnable()).start();

        manualThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (isRunning) {
                    try {
                        Thread.currentThread().sleep(100);
                        if (isNavi || isSlam || isTrack) {
                            if (isJoystickTriggered) {
                                mynpu.SetManualVel(linScale, angScale);
                                toRunOnce = true;
                            }

                            if (!isJoystickTriggered) {
                                if (toRunOnce) {
                                    mynpu.SetManualVel(0, 0);
                                    toRunOnce = false;
                                }
                            }
                        }
                        Message msg = new Message();
                        msg.what = 0;
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    } catch (Throwable e) {
                        Log.d(TAG, "Exception: " + e.toString());
                        return;
                    }
                }
            }
        });

        manualThread.start(); /* 启动线程 */

        iv_background.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (mapInfo == null)
                    return false;
                float rel = mapInfo.resolution;
                if (mynpu.isInited) {
                    switch (event.getAction() & MotionEvent.ACTION_MASK) {
                        case MotionEvent.ACTION_DOWN: //Click select map
                            if (pixelMat == null || mapInfo.offset == null)
                                return false;
                            mode = MODE.DRAG;
                            startPoint.set(event.getX(), event.getY());
                            currentMatrix.set(iv_background.getImageMatrix());

                            float x = (float) (((event.getX() + mapZoomdx) / (mapZoomScare * mapEnlargeLevel) - mapDragdx / mapEnlargeLevel) * rel / pixelMat.ratio + mapInfo.offset.x);
                            initPoseX = x;
                            setPoseX = x;
                            float y = (float) ((pixelMat.height - (event.getY() + mapZoomdy) / (mapEnlargeLevel * mapZoomScare) + mapDragdy / mapEnlargeLevel) * rel / pixelMat.ratio + mapInfo.offset.y);
                            initPoseY = y;
                            setPoseY = y;
                            int imgposeu = (int) (((event.getX() + mapZoomdx) / (mapZoomScare * mapEnlargeLevel) - mapDragdx / mapEnlargeLevel));
                            initImgposeU = imgposeu;
                            setImgposeU = imgposeu;
                            int imgposev = (int) (((event.getY() + mapZoomdy) / (mapEnlargeLevel * mapZoomScare) - mapDragdy / mapEnlargeLevel));
                            initImgposeV = imgposev;
                            setImgposeV = imgposev;

                            if (isSettingPathpose || isSettingFreePath) {
                                float yaw = poseYaw;
                                Pose3D newpose = new Pose3D(setPoseX, setPoseY, 0, 0, 0, yaw);
                                pathPose[pathPoseNum] = newpose;
                                ImgPose newimgpose = new ImgPose(setImgposeU, setImgposeV, yaw);
                                imgPathPose[pathPoseNum] = newimgpose;
                                ImgPoint newImgPoint = new ImgPoint(setImgposeU, setImgposeV);
                                imgPathPoint[pathPoseNum] = newImgPoint;
                                pathPoseNum++;
                            }
                            break;
                        case MotionEvent.ACTION_POINTER_DOWN:
                            mode = MODE.ZOOM;
                            startDis = Distance(event);
                            if (startDis > 1f) {
                                midPoint = new PointF(iv_background.getWidth() / 2, iv_background.getHeight() / 2);
                            }
                            break;
                        case MotionEvent.ACTION_MOVE:
                            if (mode == MODE.DRAG) {
                                if (isSettingInitialPose || isSettingGoalPose || isSettingPathpose || isSettingFreePath) {

                                    if (isSettingInitialPose) {
                                        double poseyaw = Math.atan2(startPoint.y - event.getY(), event.getX() - startPoint.x);   //返回的就是弧度制 不需要再转换
                                        poseYaw = (float) poseyaw;
                                        initPoseYaw = (float) poseyaw;
                                        initImgposeTheta = (float) poseyaw;
                                        redarrowYaw = initImgposeTheta;
                                    } else if (isSettingGoalPose) {
                                        double goalyaw = Math.atan2(startPoint.y - event.getY(), event.getX() - startPoint.x);
                                        poseYaw = (float) goalyaw;
                                        setPoseYaw = (float) goalyaw;
                                        setImgposeTheta = (float) goalyaw;
                                        redarrowYaw = setImgposeTheta;

                                        if (isSettingFreePath) {
                                            imgPathPose[pathPoseNum - 1].theta = poseYaw;
                                        }
                                    } else if (isSettingPathpose || isSettingFreePath) {
                                        double goalyaw = Math.atan2(startPoint.y - event.getY(), event.getX() - startPoint.x);
                                        poseYaw = (float) goalyaw;
                                        if (pathPoseNum == 0)
                                            return false;
                                        imgPathPose[pathPoseNum - 1].theta = poseYaw;
                                        redarrowYaw = poseYaw;
                                    }
                                } else {
                                    actionMove = true;
                                    float dxtmp = event.getX() - startPoint.x;
                                    float dytmp = event.getY() - startPoint.y;
                                    matrix.set(currentMatrix);
                                    matrix.postTranslate(dxtmp, dytmp);
                                }
                            }
                            if (mode == MODE.ZOOM) {

                                if (isSettingInitialPose || isSettingGoalPose || isSettingPathpose || isSettingFreePath) {
                                } else {
                                    float endDis = Distance(event);
                                    if (endDis > 1f) {
                                        float scale = endDis / startDis;
                                        float scaretmppre = mapZoomScareTemp;
                                        mapZoomScareTemp = scale;
                                        mapZoomScareTemp = mapZoomScareTemp * mapZoomScare;
                                        if (mapZoomScareTemp < 1) {
                                            mapZoomScareTemp = scaretmppre;
                                            break;
                                        }
                                        matrix.set(currentMatrix);
                                        matrix.postScale(scale, scale, midPoint.x, midPoint.y);
                                    }
                                }
                            }
                            break;
                        case MotionEvent.ACTION_POINTER_UP:
                            break;
                        case MotionEvent.ACTION_UP:
                            if (mode == mode.DRAG && actionMove == true) {
                                actionMove = false;
                                mapDragdx = mapDragdx + (event.getX() - startPoint.x) / mapZoomScareTemp;
                                mapDragdy = mapDragdy + (event.getY() - startPoint.y) / mapZoomScareTemp;
                            }
                            mapZoomScare = mapZoomScareTemp;
                            if (mode == mode.ZOOM) {
                                mapZoomdx = ((mapZoomScare - 1) * iv_background.getWidth()) / 2;
                                mapZoomdy = ((mapZoomScare - 1) * iv_background.getHeight()) / 2;
                            }
                            mode = mode.NONE;
                            break;
                    }
                }
                iv_background.setImageMatrix(matrix);
                return false;
            }

        });


        iv_background.setOnLongClickListener(new View.OnLongClickListener() {
            public boolean onLongClick(View v) {

                if (isNavi)  //Standard Navi Mode
                {
                    if (isSettingInitialPose) {
                        strSetInitialPose = "设置初始位";
                        isSettingInitialPose = false;
                        adapter_setting.getChildView(0, 0, true, tempview, tempparent);
                    }

                    if (isSettingGoalPose) {
                        strSetGoalPose = "设置目标点";
                        isSettingGoalPose = false;
                        adapter_setting.getChildView(0, 1, true, tempview, tempparent);
                    }
                }

                if (isTrack)      //AGVS Mode
                {

                    if (isSettingInitialPose) {
                        strSetInitialPose = "设置初始位";
                        isSettingInitialPose = false;
                        adapter_setting.getChildView(0, 0, true, tempview, tempparent);
                    }

                    if (isSettingGoalPose) {
                        isSettingGoalPose = false;
                        strSetGoalPose = "设置自由路径";
                        isSettingFreePath = false;
                        adapter_setting.getChildView(0, 1, true, tempview, tempparent);

                    }
                    pathPoseNum = 0;
                    if (toModifyPath || toModifyPathYaw) {
                        for (int i = 0; i < imgPathList.length; i++) {
                            for (int j = 0; j < imgPathList[i].poses.length; j++) {
                                if (Math.abs(setImgposeU - imgPathList[i].poses[j].u) < 10 && Math.abs(setImgposeV - imgPathList[i].poses[j].v) < 10) {
                                    pointIsSelected = true;
                                    Utils.showToast(getApplicationContext(), "点选中成功，请通过遥控旋钮进行微调！");

                                }

                            }
                        }
                    }
                }

                if (!mynpu.isInited) {
                    Utils.showToast(getApplicationContext(), "正在连接NPU...");
                    ConnectNPU();
                }
                return false;
            }
        });

        cb_lidar_display.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

            public void onCheckedChanged(CompoundButton buttonView,
                                         boolean isChecked) {
                // TODO Auto-generated method stub
                if (isChecked) {

                    toDisplayLidar = true;

                } else {
                    toDisplayLidar = false;
                }
            }
        });

        cb_path_display.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

            public void onCheckedChanged(CompoundButton buttonView,
                                         boolean isChecked) {
                // TODO Auto-generated method stub
                if (isChecked) {
                    toDisplayPath = true;

                } else {
                    toDisplayPath = false;
                }
            }
        });

        cb_station_display.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {

            public void onCheckedChanged(CompoundButton buttonView,
                                         boolean isChecked) {
                // TODO Auto-generated method stub
                if (isChecked) {
                    toDisplayStation = true;
                } else {
                    toDisplayStation = false;
                }
            }
        });


        sp_map_list.setOnTouchListener(new Spinner.OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (mynpu.isInited) {
                    listMapId.clear();
                    MapInfo[] newMapInfoList = mynpu.GetMapInfos();
                    if (newMapInfoList == null || newMapInfoList.length == 0)
                        return false;
                    for (int i = 0; i < newMapInfoList.length; i++) {
                        listMapId.add(newMapInfoList[i].id);
                    }
                    mapInfoList = newMapInfoList;
                    mapname = mapInfoList[0].id;
                    Log.d(TAG, "mapname:" + mapname);
                    ReadThumbnail();                  //选择地图时显示缩略图
                    Display();
                    adapterMapList.notifyDataSetChanged();
                    ibt_delete_map.setVisibility(View.VISIBLE);
                }
                return false;
            }
        });

        sp_map_list.setOnItemSelectedListener(new OnItemSelectedListener() {
            public void onItemSelected(AdapterView<?> parent,
                                       View view, int position, long id) {
                if (mynpu.isInited) {
                    if (mapInfoList == null)
                        return;
                    Spinner spinner = (Spinner) parent;
                    mapname = spinner.getSelectedItem().toString();
                    Log.d(TAG, "selectMapName:" + spinner.getSelectedItem().toString());
                    ReadThumbnail();                  //选择地图时显示缩略图
                    Display();
                    adapterMapList.notifyDataSetChanged();
                }
            }

            public void onNothingSelected(AdapterView<?> parent) {


            }
        });


        sp_path_station_list.setOnTouchListener(new Spinner.OnTouchListener() {

            @Override
            public boolean onTouch(View v, MotionEvent event) {

                if (isInited && isTrack) {
                    listPathStationId.clear();

                    ImgPath[] newImgPathList = mynpu.GetImgPaths(mapname);
                    if (newImgPathList == null)
                        return false;

                    for (int i = 0; i < newImgPathList.length; i++) {
                        listPathStationId.add(newImgPathList[i].info.id);
                    }
                    imgPathList = newImgPathList;
                    adapterPathStationList.notifyDataSetChanged();
                }


                if (isInited && isNavi) {
                    listPathStationId.clear();
                    ImgStation[] newImgStationList = mynpu.GetImgStations(mapname);
                    if (newImgStationList == null)
                        return false;
                    for (int i = 0; i < newImgStationList.length; i++) {
                        listPathStationId.add(newImgStationList[i].info.id);
                    }
                    imgStationList = newImgStationList;
                    adapterPathStationList.notifyDataSetChanged();
                }


                return false;
            }
        });


        // Get the iv_background width and height to use it calculate the scale
        ViewTreeObserver vto = iv_background.getViewTreeObserver();
        vto.addOnGlobalLayoutListener(new OnGlobalLayoutListener() {
            @Override
            public void onGlobalLayout() {
                iv_background.getViewTreeObserver().removeGlobalOnLayoutListener(this);
                canvasWidth = iv_background.getWidth();
                canvasHeight = iv_background.getHeight();
            }
        });
        elv_operate = (ExpandableListView) findViewById(R.id.elv);
        adapter_operate = new MyExpandableListAdapterOperate(operateTitle, operateItemData, this);
        elv_operate.setAdapter(adapter_operate);

        elv_setting = (ExpandableListView) findViewById(R.id.elv_setting);
        adapter_setting = new MyExpandableListAdapterSetting(settingTitle, settingItemData, this);
        elv_setting.setAdapter(adapter_setting);

        elv_function = (ExpandableListView) findViewById(R.id.elv_function);
        adapter_function = new MyExpandableListAdapterFunction(functionTitle, functionItemData, this);
        elv_function.setAdapter(adapter_function);

        //收缩
        elv_operate.setOnGroupCollapseListener(new OnGroupCollapseListener() {

            @Override
            public void onGroupCollapse(int groupPosition) {
                Utils.setListViewHeightBasedOnChildren(elv_operate);
            }
        });
        //伸展
        elv_operate.setOnGroupExpandListener(new OnGroupExpandListener() {
            @Override
            public void onGroupExpand(int groupPosition) {
                Utils.setListViewHeightBasedOnChildren(elv_operate);
            }
        });
        //子条目点击
        elv_operate.setOnChildClickListener(new OnChildClickListener() {
            @Override
            public boolean onChildClick(ExpandableListView parent, View v,
                                        int groupPosition, int childPosition, long id) {

                if (isTrack) {

                    if (groupPosition == 0 && childPosition == 0) {

                        TrackPath();
                        adapter_operate.getChildView(0, 0, true, v, parent);
                    }

                    if (groupPosition == 1 && childPosition == 0) {

                        SavePath();
                        adapter_operate.getChildView(1, 0, false, v, parent);
                    }

                    if (groupPosition == 1 && childPosition == 1) {

                        SetStationPath();
                    }

                    if (groupPosition == 1 && childPosition == 2) {

                        SaveStationPath();
                    }

                    if (groupPosition == 2 && childPosition == 0) {

                        ModifyPath();
                        adapter_operate.getChildView(2, 0, false, v, parent);
                    }

                    if (groupPosition == 2 && childPosition == 1) {

                        ModifyPathYaw();
                        adapter_operate.getChildView(2, 1, false, v, parent);
                    }

                    if (groupPosition == 2 && childPosition == 2) {
                        DeletePath();
                        adapter_operate.getChildView(2, 2, false, v, parent);
                    }
                }

                if (isNavi) {
                    if (groupPosition == 0 && childPosition == 0) {
                        SaveStation();
                    }

                    if (groupPosition == 0 && childPosition == 1) {
                        DeleteStation();
                    }

                    if (groupPosition == 0 && childPosition == 2) {

                        GotoStation();
                    }
                }

                return false;
            }
        });


        //收缩
        elv_setting.setOnGroupCollapseListener(new OnGroupCollapseListener() {

            @Override
            public void onGroupCollapse(int groupPosition) {
                Utils.setListViewHeightBasedOnChildren(elv_setting);
            }
        });
        //伸展
        elv_setting.setOnGroupExpandListener(new OnGroupExpandListener() {
            @Override
            public void onGroupExpand(int groupPosition) {
                Utils.setListViewHeightBasedOnChildren(elv_setting);

            }
        });

        elv_setting.setOnChildClickListener(new OnChildClickListener() {
            @Override
            public boolean onChildClick(ExpandableListView parent, View v,
                                        int groupPosition, int childPosition, long id) {

                if (isNavi) {
                    if (groupPosition == 0 && childPosition == 0) {
                        if (!isSettingGoalPose) {
                            SetInitPose();
                            tempparent = parent;
                            tempview = v;
                            adapter_setting.getChildView(0, 0, true, v, parent);
                        }
                    }

                    if (groupPosition == 0 && childPosition == 1) {
                        if (!isSettingInitialPose) {
                            SetGoalPose();
                            tempparent = parent;
                            tempview = v;
                            adapter_setting.getChildView(0, 1, true, v, parent);
                        }
                    }
                }

                if (isTrack) {
                    if (groupPosition == 0 && childPosition == 0) {
                        if (!isSettingGoalPose) {
                            SetInitPose();
                            tempparent = parent;
                            tempview = v;
                            adapter_setting.getChildView(0, 0, true, v, parent);
                        }
                    }


                    if (groupPosition == 0 && childPosition == 1) {
                        if (!isSettingInitialPose) {
                            tempparent = parent;
                            tempview = v;
                            SetFreePath();
                        }
                    }
                }

                return false;
            }
        });


        elv_function.setOnGroupCollapseListener(new OnGroupCollapseListener() {

            @Override
            public void onGroupCollapse(int groupPosition) {
                Utils.setListViewHeightBasedOnChildren(elv_function);
            }
        });
        //伸展
        elv_function.setOnGroupExpandListener(new OnGroupExpandListener() {
            @Override
            public void onGroupExpand(int groupPosition) {
                Utils.setListViewHeightBasedOnChildren(elv_function);
            }
        });

        elv_function.setOnChildClickListener(new OnChildClickListener() {
            @Override
            public boolean onChildClick(ExpandableListView parent, View v,
                                        int groupPosition, int childPosition, long id) {

                if (groupPosition == 0 && childPosition == 0 && !isSlam) {
                    TrackNavi();
                }

                if (groupPosition == 0 && childPosition == 1 && !isNavi && !isTrack) {
                    SlamMap();
                }
                return false;
            }
        });

        Utils.showToast(getApplicationContext(), "正在连接NPU...");
        ConnectNPU();
    }

    private static class MyHandler extends Handler {
        WeakReference<WizRoboNpu> weakReference;

        public MyHandler(WizRoboNpu activity) {
            weakReference = new WeakReference<WizRoboNpu>(activity);
        }

        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            if (weakReference.get() != null) {
                // update android ui
            }
        }
    }

    private static class MyRunnable implements Runnable {

        @Override
        public void run() {

            while (true) {
                imgmapHandler.sendEmptyMessage(MESSAGECODE);
                try {

                    Thread.currentThread().sleep(1000);
                    if (toGetImgMap) {
                        if (!isTimeout) {
                            if (mynpu.isInited)
                                imgMap = mynpu.GetCurrentImgMap();
                        }
                    }

                    if (isNavi || isSlam || isTrack) {
                        if (!isTimeout) {
                            if (mynpu.isInited) {
                                cmdVel = mynpu.GetCmdVel();
                                actVel = mynpu.GetActVel();
                                cmdMotorSpd = mynpu.GetCmdMotorSpd();
                                actMotorSpd = mynpu.GetActMotorSpd();
                                naviState = mynpu.GetNaviState();
                                actPose = mynpu.GetCurrentPose();
                                actImgPose = mynpu.GetCurrentImgPose();
                                imglidarscandata = mynpu.GetImgLidarScan();
                                FootprintVerticles = mynpu.GetFootprintImgVertices();
                                if (WizRoboNpu.isNavi || WizRoboNpu.isTrack) {
                                    cmdImgPath = mynpu.GetCmdImgPath();
                                }
                            }
                        }
                    }
                } catch (InterruptedException e) {
                    e.printStackTrace();
                } catch (Throwable e) {
                    e.printStackTrace();
                }
                imgmapHandler.sendEmptyMessage(MESSAGECODE);
            }
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        //如果参数为null的话，会将所有的Callbacks和Messages全部清除掉。
        handler.removeCallbacksAndMessages(null);
        handler_display_points.removeCallbacksAndMessages(null);
        handler_update_ui.removeCallbacksAndMessages(null);
        imgmapHandler.removeCallbacksAndMessages(null);
    }

    public void toUpDianji() {
        dianjiMap.clear();
        for (int i = 0; i < 16; i++) {
            dianjiMap.put("#" + i, "P1200");
        }
    }

    public String getCmd() {
        StringBuffer sb = new StringBuffer();
        Iterator<String> iter = dianjiMap.keySet().iterator();
        while (iter.hasNext()) {
            String key = iter.next();
            String value = dianjiMap.get(key);
            sb.append(key + " " + value + " ");
        }
        sb.append("T 500\r");
        return sb.toString();
    }

    public void toDownDianji() {
        dianjiMap.clear();
        for (int i = 0; i < 16; i++) {
            dianjiMap.put("#" + i, "P1400");
        }
    }


    @Override
    protected void onStart() {
        ConnectivityManager manager = (ConnectivityManager) this
                .getSystemService(Context.CONNECTIVITY_SERVICE);
        NetworkInfo wifiNetworkInfo = manager.getNetworkInfo(ConnectivityManager.TYPE_WIFI);
        if (wifiNetworkInfo.isConnected()) {
            String wserviceName = Context.WIFI_SERVICE;
            WifiManager wm = (WifiManager) getSystemService(wserviceName);
            WifiInfo info1 = wm.getConnectionInfo();
            wifiIsConnected = true;
            Toast.makeText(WizRoboNpu.this, "wifi名称：" + info1.getSSID(), Toast.LENGTH_LONG).show();

        } else {
            AlertDialog.Builder builder = new AlertDialog.Builder(WizRoboNpu.this, AlertDialog.THEME_DEVICE_DEFAULT_DARK);
            builder.setTitle("wifi未连接");
            builder.setMessage("请点击确定键进行设置！");
            builder.setPositiveButton("确定",
                    new DialogInterface.OnClickListener() {
                        public void onClick(DialogInterface dialog, int which) {
                            Intent i = new Intent();
                            if (android.os.Build.VERSION.SDK_INT >= 11) {
                                //Honeycomb
                                i.setClassName("com.android.settings", "com.android.settings.Settings$WifiSettingsActivity");
                            } else {
                                //other versions
                                i.setClassName("com.android.settings"
                                        , "com.android.settings.wifi.WifiSettings");
                            }
                            startActivity(i);
                            dialog.cancel();
                        }
                    });

            builder.setNegativeButton("取消",
                    new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            dialog.cancel();
                        }
                    });
            builder.show();
        }
        super.onStart();
    }


    private void SetInitPose() {
        if (isNavi || isTrack) {
            if (!isSettingInitialPose && !isSettingGoalPose && naviState != NaviState.ACTIVE) {
                isSettingInitialPose = true;
                strSetInitialPose = "确定";

                initImgposeU = 0;
                initImgposeV = 0;
                initImgposeTheta = 0;
                pathPoseNum = 0;
                isSettingFreePath = false;
            } else if (isSettingInitialPose && !isSettingGoalPose && naviState != NaviState.ACTIVE) {
                isSettingInitialPose = false;
                strSetInitialPose = "设置初始位";
                if (isPoseMode) {
                    Pose3D pose = new Pose3D(initPoseX, initPoseY, 0, 0, 0, initPoseYaw);
                    mynpu.SetInitPose(pose);
                }
                if (isImgPoseMode) {
                    ImgPose pose = new ImgPose(initImgposeU, initImgposeV, initImgposeTheta);
                    mynpu.SetInitImgPose(pose);
                }
            }
        }
    }


    private void SetGoalPose() {
        if (isNavi)  //Standard Navi Mode
        {
            if (!isSettingGoalPose && !isSettingInitialPose) {
                isSettingGoalPose = true;
                strSetGoalPose = "确定";
                setImgposeU = 0;
                setImgposeV = 0;
                setImgposeTheta = 0;
            } else if (isSettingGoalPose && !isSettingInitialPose) {
                isSettingGoalPose = false;
                strSetGoalPose = "设置目标点";
                if (isPoseMode) {
                    Pose3D pose = new Pose3D(setPoseX, setPoseY, 0, 0, 0, setPoseYaw);
                    mynpu.GotoGoal(pose);
                }
                if (isImgPoseMode) {
                    ImgPose pose = new ImgPose(setImgposeU, setImgposeV, setImgposeTheta);
                    mynpu.GotoImgGoal(pose);
                }
                Log.d(TAG, "GotoPose");
            }
        }
    }


    private void SetFreePath() {
        if (isTrack)      //AGVS Mode
        {
            if (!isSettingGoalPose && !isSettingInitialPose) {
                isSettingFreePath = true;
                isSettingGoalPose = true;
                strSetGoalPose = "确定";
                setImgposeU = 0;
                setImgposeV = 0;
                setImgposeTheta = 0;
                adapter_setting.getChildView(0, 1, true, tempview, tempparent);
            } else if (isSettingGoalPose && !isSettingInitialPose) {
                isSettingGoalPose = false;
                isSettingFreePath = false;
                strSetGoalPose = "设置自由路径";
                ImgPose[] newimgpose = new ImgPose[pathPoseNum];
                for (int i = 0; i < pathPoseNum; i++) {
                    newimgpose[i] = imgPathPose[i];
                    Log.d(TAG, "imagepose" + newimgpose[i].theta);
                }
                ImgPoint[] newPointList = new ImgPoint[pathPoseNum];
                for (int i = 0; i < pathPoseNum; i++) {
                    newPointList[i] = imgPathPoint[i];
                }

                if (pathPoseNum == 0) {
                    adapter_setting.getChildView(0, 1, true, tempview, tempparent);
                    return;
                }
                mynpu.FollowTempImgPath(newimgpose);
                pathPoseNum = 0;
                adapter_setting.getChildView(0, 1, true, tempview, tempparent);
            }
        }
    }

    private void Navi() {
        if (mynpu.isInited) {

            if (!isNavi && !isTrack && !isSlam) {

                try {

                    if (mynpu.GetActionState() == ActionState.IDLE_ACTION) {
                        if (mapname.length() == 0) {
                            Utils.showToast(getApplicationContext(), "请先选择地图");
                            return;
                        }
                        mynpu.SelectMap(mapname);
                        mynpu.StartNavi(NaviMode.P2P_NAVI);
                    } else {

                        mynpu.SelectMap(mapname);
                    }

                    ibt_delete_map.setVisibility(View.INVISIBLE);
                    cb_station_display.setVisibility(View.VISIBLE);
                    cb_lidar_display.setVisibility(View.VISIBLE);
                    cb_path_display.setVisibility(View.VISIBLE);
                    ibt_pause.setVisibility(View.VISIBLE);
                    ibt_cancel.setVisibility(View.VISIBLE);
                    bt_reboot.setVisibility(View.VISIBLE);
                    sp_map_list.setEnabled(false);


                    imgMap = mynpu.GetCurrentImgMap();
                    Utils.Delay(1000);
                    ReadImgMap();
                    Utils.Delay(2000);
                    isNavi = true;
                    strNaviTrack = "停止导航";
                    adapter_function.getChildView(0, 0, true, track_navi_view, track_navi_parent);
                    HashMap<String, List<String>> itemData1 = ExpandableData.getOperateData();
                    List<String> title1 = new ArrayList<String>(itemData1.keySet());
                    adapter_operate = new MyExpandableListAdapterOperate(title1, itemData1, this);
                    elv_operate.setAdapter(adapter_operate);
                    strSetGoalPose = "设置目标点";
                    HashMap<String, List<String>> itemData2 = ExpandableData.getSettingData();
                    List<String> title2 = new ArrayList<String>(itemData2.keySet());
                    adapter_setting = new MyExpandableListAdapterSetting(title2, itemData2, this);
                    elv_setting.setAdapter(adapter_setting);

                    sv_control.setVisibility(View.VISIBLE);
                    listPathStationId.clear();
                    adapterPathStationList.notifyDataSetChanged();
                } catch (Exception e) {
                    ExceptionAlert(e);
                }
            } else if (isNavi && !isTrack && !isSlam) {
                cb_station_display.setVisibility(View.INVISIBLE);
                cb_lidar_display.setVisibility(View.INVISIBLE);
                cb_path_display.setVisibility(View.INVISIBLE);
                bt_reboot.setVisibility(View.INVISIBLE);
                ibt_pause.setVisibility(View.INVISIBLE);
                ibt_cancel.setVisibility(View.INVISIBLE);
                mynpu.StopNavi();
                Utils.Delay(2000);
                isNavi = false;
                strNaviTrack = "开始导航";
                adapter_function.getChildView(0, 0, true, track_navi_view, track_navi_parent);
                sp_map_list.setEnabled(true);
                sv_control.setVisibility(View.INVISIBLE);
            }
        }
    }

    private void Track() {
        if (mynpu.isInited) {

            if (!isTrack && !isNavi && !isSlam) {
                try {

                    if (mynpu.GetActionState() == ActionState.IDLE_ACTION) {
                        if (mapname!=null&&mapname.length()!=0) {
                            Utils.showToast(getApplicationContext(), "请先选择地图");
                            return;
                        }
                        mynpu.SelectMap(mapname);
                        mynpu.StartNavi(NaviMode.PF_NAVI);
                    } else {
                        mynpu.SelectMap(mapname);
                    }

                    ibt_delete_map.setVisibility(View.INVISIBLE);
                    cb_station_display.setVisibility(View.VISIBLE);
                    cb_lidar_display.setVisibility(View.VISIBLE);
                    cb_path_display.setVisibility(View.VISIBLE);
                    bt_reboot.setVisibility(View.VISIBLE);
                    ibt_pause.setVisibility(View.VISIBLE);
                    ibt_cancel.setVisibility(View.VISIBLE);
                    sp_map_list.setEnabled(false);
                    imgMap = mynpu.GetCurrentImgMap();

                    Utils.Delay(1000);
                    ReadImgMap();
                    Utils.Delay(3000);
                    isTrack = true;
                    strNaviTrack = "停止导航";
                    adapter_function.getChildView(0, 0, true, track_navi_view, track_navi_parent);

                    HashMap<String, List<String>> itemData1 = ExpandableData.getOperateData();
                    List<String> title1 = new ArrayList<String>(itemData1.keySet());
                    adapter_operate = new MyExpandableListAdapterOperate(title1, itemData1, this);
                    elv_operate.setAdapter(adapter_operate);

                    strSetGoalPose = "设置自由路径";
                    HashMap<String, List<String>> itemData2 = ExpandableData.getSettingData();
                    List<String> title2 = new ArrayList<String>(itemData2.keySet());
                    adapter_setting = new MyExpandableListAdapterSetting(title2, itemData2, this);
                    elv_setting.setAdapter(adapter_setting);

                    sv_control.setVisibility(View.VISIBLE);
                    listPathStationId.clear();
                    ImgPath[] newImgPathList = mynpu.GetImgPaths(mapname);
                    if (newImgPathList == null)
                        return;
                    for (int i = 0; i < newImgPathList.length; i++) {
                        listPathStationId.add(newImgPathList[i].info.id);
                    }
                    imgPathList = newImgPathList;
                    adapterPathStationList.notifyDataSetChanged();
                } catch (Exception e) {
                    ExceptionAlert(e);
                }
            } else if (isTrack && !isNavi && !isSlam) {
                mynpu.StopNavi();
                Utils.Delay(2000);
                isTrack = false;
                strNaviTrack = "开始导航";
                adapter_function.getChildView(0, 0, true, track_navi_view, track_navi_parent);
                sp_map_list.setEnabled(true);
                ibt_pause.setVisibility(View.INVISIBLE);
                ibt_cancel.setVisibility(View.INVISIBLE);
                bt_reboot.setVisibility(View.INVISIBLE);

                sv_control.setVisibility(View.INVISIBLE);
                cb_station_display.setVisibility(View.INVISIBLE);
                cb_lidar_display.setVisibility(View.INVISIBLE);
                cb_path_display.setVisibility(View.INVISIBLE);
            }
        }
    }

    private void TrackNavi() {
        if (!isNavi && !isTrack) {
            //若需弹窗，请参照源代码，搜索 循迹导航
            if (isInited) {
                Utils.showToast(getApplicationContext(), "正在启动循迹导航...");
                Track();
            }
        } else if (isNavi) {
            Navi();
        } else {
            Track();
        }

    }

    private void Slam() {
        if (mynpu.isInited) {

            if (!isSlam && !isNavi && !isTrack) {
                if (mynpu.GetActionState() == ActionState.SLAM_ACTION) {
                } else {
                    mynpu.StartSlam(slamMode);
                    Utils.Delay(500);
                    Log.d(TAG, "start slam");
                }

                sp_map_list.setEnabled(false);
                ibt_delete_map.setVisibility(View.INVISIBLE);
                Utils.Delay(2000);
                isSlam = true;
                strSlam = "停止建图";
                adapter_function.getChildView(0, 1, true, slam_view, slam_parent);

                toGetImgMap = true;
            } else if (isSlam && !isNavi && !isTrack) {

                final EditText et_mapnamex = new EditText(WizRoboNpu.this);
                final AlertDialog dialog = new AlertDialog.Builder(WizRoboNpu.this)
                        .setTitle(R.string.str_notice)
                        // .setIcon(R.drawable.warming)
                        .setMessage(R.string.str_notice_inputmapname)
                        //.setMessage("不能包含@#￥%&*等特殊字符！")
                        .setView(et_mapnamex)
                        .setNegativeButton("取消", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialog, int which) {
                                dialog.dismiss();
                            }
                        })
                        .setPositiveButton("确定", null)
                        .setCancelable(true)
                        .create();
                dialog.show();

                //为了避免点击 positive 按钮后直接关闭 dialog,把点击事件拿出来设置
                dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(new OnClickListener() {

                                                                                     @Override
                                                                                     public void onClick(View v) {
                                                                                         boolean a;
                                                                                         String name = et_mapnamex.getText().toString();
                                                                                         if (name == null || name.length() <= 0) {
                                                                                             mynpu.StopSlam("");
                                                                                             dialog.dismiss();
                                                                                         } else {
                                                                                             a = Utils.SpecialSymbols(name);
                                                                                             if (a) {
                                                                                                 Utils.showToast(getApplicationContext(), "不能包含*&%$#@!等特殊字符");
                                                                                                 return;
                                                                                             }
                                                                                             mynpu.StopSlam(name);
                                                                                             Log.d(TAG, "stop slam");
                                                                                             dialog.dismiss();
                                                                                         }

                                                                                         Utils.Delay(2000);
                                                                                         isSlam = false;
                                                                                         strSlam = "开始建图";
                                                                                         adapter_function.getChildView(0, 1, true, slam_view, slam_parent);
                                                                                         sp_map_list.setEnabled(true);
                                                                                         toGetImgMap = false;
                                                                                     }
                                                                                 }
                );

            }
        }
    }

    private void SlamMap() {

        if (!isSlam) {
            //若需弹窗，请参照源代码，搜索 图优化
            if (isInited) {
                slamMode = SlamMode.ICP_SLAM;
                Slam();
            }
        }

    }

    private void SavePath() {
        if (mynpu.isInited) {

            if (!isSettingPathpose && !isSettingStationPath) {
                isSettingPathpose = true;
                setImgposeU = 0;
                setImgposeV = 0;
                setImgposeTheta = 0;
                Utils.showToast(getApplicationContext(), "可通过遥控改变点的朝向角！");
                strSavePath = "保存";
            } else if (isSettingPathpose && !isSettingStationPath) {
                isSettingPathpose = false;
                strSavePath = "自动添加路径";
                if (pathPoseNum == 0)
                    return;
                final EditText et_pathnamex = new EditText(WizRoboNpu.this);
                final AlertDialog dialog = new AlertDialog.Builder(WizRoboNpu.this)
                        .setTitle(R.string.str_notice)
                        .setMessage("请输入路径名称！")
                        .setView(et_pathnamex)
                        .setNegativeButton("取消", new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialog, int which) {
                                dialog.dismiss();
                            }
                        })
                        .setPositiveButton("确定", null)
                        .setCancelable(true)
                        .create();
                dialog.show();

                //为了避免点击 positive 按钮后直接关闭 dialog,把点击事件拿出来设置
                dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(new OnClickListener() {

                                                                                     @Override
                                                                                     public void onClick(View v) {
                                                                                         boolean a;
                                                                                         String name = et_pathnamex.getText().toString();
                                                                                         if (name == null || name.length() <= 0) {

                                                                                         } else {
                                                                                             a = Utils.SpecialSymbols(name);
                                                                                             if (a) {
                                                                                                 Utils.showToast(getApplicationContext(), "不能包含*&%$#@!等特殊字符");
                                                                                                 return;
                                                                                             }
                                                                                             dialog.dismiss();
                                                                                         }
                                                                                         Log.d(TAG, "IS SAVING PATH");
                                                                                         listPathStationId.clear();
                                                                                         ImgPath[] newImgPathList = mynpu.GetImgPaths(mapname);
                                                                                         List<ImgPath> imgPaths = new ArrayList<ImgPath>(0);
                                                                                         if (newImgPathList != null) {
                                                                                             for (int i = 0; i < newImgPathList.length; i++) {
                                                                                                 imgPaths.add(newImgPathList[i]);
                                                                                             }
                                                                                         }
                                                                                         PathInfo newpathinfo = new PathInfo();
                                                                                         ImgPose[] newimgpose = new ImgPose[pathPoseNum];
                                                                                         ImgPath newpath = new ImgPath();
                                                                                         newpathinfo.map_id = mapname;
                                                                                         newpathinfo.id = name;
                                                                                         newpathinfo.length = 0;
                                                                                         newpathinfo.pose_num = pathPoseNum;

                                                                                         for (int i = 0; i < pathPoseNum; i++) {
                                                                                             newimgpose[i] = imgPathPose[i];
                                                                                             Log.d(TAG, "imgpathpose:" + imgPathPose[i].theta);
                                                                                         }
                                                                                         newpath.info = newpathinfo;
                                                                                         newpath.poses = newimgpose;

                                                                                         imgPaths.add(newpath);


                                                                                         ImgPath[] newImgPathList1 = new ImgPath[imgPaths.size()];
                                                                                         for (int i = 0; i < imgPaths.size(); i++) {
                                                                                             newImgPathList1[i] = imgPaths.get(i);
                                                                                         }
                                                                                         mynpu.SetImgPaths(mapname, newImgPathList1);

                                                                                         for (int i = 0; i < newImgPathList1.length; i++) {
                                                                                             listPathStationId.add(newImgPathList1[i].info.id);
                                                                                         }
                                                                                         imgPathList = newImgPathList1;
                                                                                         adapterPathStationList.notifyDataSetChanged();
                                                                                         pathPoseNum = 0;
                                                                                         Utils.showToast(getApplicationContext(), "添加成功！");
                                                                                     }
                                                                                 }
                );

            }
        }
    }

    private void SaveStation() {
        if (isInited) {
            final EditText et_stationnamex = new EditText(WizRoboNpu.this);
            final AlertDialog dialog = new AlertDialog.Builder(WizRoboNpu.this)
                    .setTitle(R.string.str_notice)
                    .setMessage("请输入站点名称！")
                    .setView(et_stationnamex)
                    .setNegativeButton("取消", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            dialog.dismiss();
                        }
                    })
                    .setPositiveButton("确定", null)
                    .setCancelable(true)
                    .create();
            dialog.show();
            //为了避免点击 positive 按钮后直接关闭 dialog,把点击事件拿出来设置
            dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(new OnClickListener() {

                                                                                 @Override
                                                                                 public void onClick(View v) {
                                                                                     boolean a;
                                                                                     String name = et_stationnamex.getText().toString();
                                                                                     if (name == null || name.length() <= 0) {

                                                                                     } else {
                                                                                         a = Utils.SpecialSymbols(name);
                                                                                         if (a) {

                                                                                             Utils.showToast(getApplicationContext(), "不能包含*&%$#@!等特殊字符");
                                                                                             return;
                                                                                         }
                                                                                         dialog.dismiss();
                                                                                     }


                                                                                     listPathStationId.clear();
                                                                                     ImgStation[] newImgStationList = mynpu.GetImgStations(mapname);
                                                                                     List<ImgStation> imgStations = new ArrayList<ImgStation>(0);

                                                                                     if (newImgStationList == null) {
                                                                                         Log.d(TAG, "IS Adding");
                                                                                     } else {
                                                                                         for (int i = 0; i < newImgStationList.length; i++) {
                                                                                             imgStations.add(newImgStationList[i]);
                                                                                         }
                                                                                     }
                                                                                     StationInfo newstationinfo = new StationInfo();
                                                                                     ImgPose newimgpose = new ImgPose();
                                                                                     ImgStation newstation = new ImgStation();
                                                                                     newstationinfo.map_id = mapname;
                                                                                     newstationinfo.id = name;
                                                                                     newstationinfo.type = StationType.USER_DEFINED;
                                                                                     newstationinfo.artag_id = 0;
                                                                                     newimgpose = actImgPose;
                                                                                     newstation.info = newstationinfo;
                                                                                     newstation.pose = newimgpose;

                                                                                     imgStations.add(newstation);


                                                                                     ImgStation[] newImgStationList1 = new ImgStation[imgStations.size()];
                                                                                     for (int i = 0; i < imgStations.size(); i++) {
                                                                                         newImgStationList1[i] = imgStations.get(i);
                                                                                     }
                                                                                     mynpu.SetImgStations(mapname, newImgStationList1);

                                                                                     for (int i = 0; i < newImgStationList1.length; i++) {
                                                                                         listPathStationId.add(newImgStationList1[i].info.id);
                                                                                     }
                                                                                     imgStationList = newImgStationList1;
                                                                                     adapterPathStationList.notifyDataSetChanged();
                                                                                     Utils.showToast(getApplicationContext(), "添加成功！");

                                                                                 }
                                                                             }
            );
        }
    }

    private OnClickListener Ibt_DeleteMap_OnClick = new OnClickListener() {
        public void onClick(View v) {

            if (mynpu.isInited) {
                if (mapInfoList == null)
                    return;
                new AlertDialog.Builder(WizRoboNpu.this, AlertDialog.THEME_DEVICE_DEFAULT_DARK)
                        .setTitle(R.string.str_warming)
                        //.setIcon(R.drawable.warming)
                        .setMessage("是否删除地图:" + mapname)

                        .setPositiveButton(R.string.str_ok,
                                new DialogInterface.OnClickListener() {
                                    public void onClick(
                                            DialogInterface dialoginterface,
                                            int i) {
                                        if (isInited) {
                                            listMapId.clear();
                                            MapInfo[] newMapInfoList = mynpu.GetMapInfos();
                                            List<MapInfo> mapInfos = new ArrayList<MapInfo>(0);
                                            if (newMapInfoList == null || newMapInfoList.length == 0)
                                                return;
                                            for (int j = 0; j < newMapInfoList.length; j++) {
                                                mapInfos.add(newMapInfoList[j]);
                                            }

                                            mapInfos.remove(sp_map_list.getSelectedItemPosition());
                                            MapInfo[] newMapInfoList1 = new MapInfo[mapInfos.size()];
                                            for (int j = 0; j < mapInfos.size(); j++) {
                                                newMapInfoList1[j] = mapInfos.get(j);
                                            }
                                            mynpu.SetMapInfos(newMapInfoList1);

                                            for (int j = 0; j < newMapInfoList1.length; j++) {
                                                listMapId.add(newMapInfoList1[j].id);
                                            }
                                            mapInfoList = newMapInfoList1;
                                            adapterMapList.notifyDataSetChanged();
                                            Utils.showToast(getApplicationContext(), "地图：" + mapname + "删除中...");
                                        }
                                    }
                                })
                        .setNegativeButton(R.string.str_no,
                                new DialogInterface.OnClickListener() {
                                    public void onClick(
                                            DialogInterface dialoginterface,
                                            int i) {
                                    }
                                }).show();

            }

        }
    };

    private OnClickListener Ibt_Pause_OnClick = new OnClickListener() {
        public void onClick(View v) {

            if (mynpu.isInited) {
                if (!pause) {
                    ibt_pause.setImageResource(R.drawable.continue2);
                    mynpu.PauseTask();
                    pause = true;
                } else {
                    ibt_pause.setImageResource(R.drawable.stop1);
                    mynpu.ContinueTask();
                    pause = false;
                }
            }
        }
    };

    private OnClickListener Ibt_Cancel_OnClick = new OnClickListener() {
        public void onClick(View v) {
            if (mynpu.isInited) {

                mynpu.CancelTask();

            }

        }
    };

    private void DeletePath() {
        if (isInited && isTrack) {

            new AlertDialog.Builder(WizRoboNpu.this)
                    .setTitle(R.string.str_warming)
                    .setMessage("是否删除该路径")
                    .setPositiveButton(R.string.str_ok,
                            new DialogInterface.OnClickListener() {
                                public void onClick(
                                        DialogInterface dialoginterface,
                                        int i) {
                                    listPathStationId.clear();
                                    ImgPath[] newImgPathList = mynpu.GetImgPaths(mapname);
                                    List<ImgPath> imgPaths = new ArrayList<ImgPath>(0);
                                    if (newImgPathList == null || newImgPathList.length == 0)
                                        return;
                                    for (int j = 0; j < newImgPathList.length; j++) {
                                        imgPaths.add(newImgPathList[j]);
                                    }
                                    imgPaths.remove(sp_path_station_list.getSelectedItemPosition());
                                    ImgPath[] newImgPathList1 = new ImgPath[imgPaths.size()];
                                    for (int j = 0; j < imgPaths.size(); j++) {
                                        newImgPathList1[j] = imgPaths.get(j);
                                    }
                                    mynpu.SetImgPaths(mapname, newImgPathList1);

                                    for (int j = 0; j < newImgPathList1.length; j++) {
                                        listPathStationId.add(newImgPathList1[j].info.id);
                                    }
                                    imgPathList = newImgPathList1;
                                    adapterPathStationList.notifyDataSetChanged();
                                    Utils.showToast(getApplicationContext(), "删除中...");
                                }
                            })
                    .setNegativeButton(R.string.str_no,
                            null).show();
        }

    }

    private void DeleteStation() {
        if (mynpu.isInited) {

            new AlertDialog.Builder(WizRoboNpu.this, AlertDialog.THEME_DEVICE_DEFAULT_DARK)
                    .setTitle(R.string.str_warming)
                    //.setIcon(R.drawable.warming)
                    .setMessage("是否删除该站点")
                    .setPositiveButton(R.string.str_ok,
                            new DialogInterface.OnClickListener() {
                                public void onClick(
                                        DialogInterface dialoginterface,
                                        int i) {
                                    listPathStationId.clear();
                                    ImgStation[] newImgStationList = mynpu.GetImgStations(mapname);
                                    List<ImgStation> imgStations = new ArrayList<ImgStation>(0);
                                    if (newImgStationList == null || newImgStationList.length == 0)
                                        return;
                                    for (int j = 0; j < newImgStationList.length; j++) {
                                        imgStations.add(newImgStationList[j]);
                                    }
                                    imgStations.remove(sp_path_station_list.getSelectedItemPosition());
                                    ImgStation[] newImgStationList1 = new ImgStation[imgStations.size()];
                                    for (int j = 0; j < imgStations.size(); j++) {
                                        newImgStationList1[j] = imgStations.get(j);
                                    }
                                    mynpu.SetImgStations(mapname, newImgStationList1);

                                    for (int j = 0; j < newImgStationList1.length; j++) {
                                        listPathStationId.add(newImgStationList1[j].info.id);
                                    }
                                    imgStationList = newImgStationList1;
                                    adapterPathStationList.notifyDataSetChanged();
                                    Utils.showToast(getApplicationContext(), "删除中...");


                                }
                            })
                    .setNegativeButton(R.string.str_no,
                            new DialogInterface.OnClickListener() {
                                public void onClick(
                                        DialogInterface dialoginterface,
                                        int i) {
                                }
                            }).show();

        }
    }

    private void GotoStation() {
        if (mynpu.isInited) {
            if (imgStationList == null || imgStationList.length == 0)
                return;
            mynpu.GotoStation(mapname, sp_path_station_list.getSelectedItem().toString());
        }
    }

    private void ModifyPath() {
        if (mynpu.isInited) {
            if (!toModifyPath && !toModifyPathYaw) {
                toModifyPath = true;
                strModifyPathXY = "保存";
                Utils.showToast(getApplicationContext(), "请在地图长按需要修改的站点！");
            } else if (toModifyPath && !toModifyPathYaw) {
                toModifyPath = false;
                strModifyPathXY = "修改XY值";
                if (pointIsSelected) {
                    pointIsSelected = false;
                    mynpu.SetImgPaths(mapname, imgPathList);
                    Utils.showToast(getApplicationContext(), "修改成功！");
                } else {

                }


            }
        }
    }

    private void ModifyPathYaw() {
        if (mynpu.isInited) {
            if (!toModifyPathYaw && !toModifyPath) {
                toModifyPathYaw = true;
                strModifyPathYaw = "保存";
                Utils.showToast(getApplicationContext(), "请在地图长按需要修改的站点！");
            } else if (toModifyPathYaw && !toModifyPath) {
                toModifyPathYaw = false;
                strModifyPathYaw = "修改角度";
                if (pointIsSelected == true) {
                    pointIsSelected = false;
                    mynpu.SetImgPaths(mapname, imgPathList);
                    Utils.showToast(getApplicationContext(), "修改成功");
                } else {

                }
            }
        }
    }

    private void SetStationPath() {
        if (mynpu.isInited && !isSettingPathpose) {
            stationPathPose[stationPathNum] = actPose;
            Log.d(TAG, "station:" + actPose.x + ", " + actPose.y + ", " + actPose.yaw);
            stationPathNum++;
            Utils.showToast(getApplicationContext(), "已添加" + stationPathNum + "个点！");
            isSettingStationPath = true;
        }
    }

    private void SaveStationPath() {
        if (mynpu.isInited && isSettingStationPath) {

            final EditText et_pathnamex = new EditText(WizRoboNpu.this);
            final AlertDialog dialog = new AlertDialog.Builder(WizRoboNpu.this)
                    .setTitle(R.string.str_notice)
                    .setMessage("请输入路径名称！")
                    .setView(et_pathnamex)
                    .setNegativeButton("取消", new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            dialog.dismiss();
                        }
                    })
                    .setPositiveButton("确定", null)
                    .setCancelable(true)
                    .create();
            dialog.show();

            //为了避免点击 positive 按钮后直接关闭 dialog,把点击事件拿出来设置
            dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener(new OnClickListener() {

                                                                                 @Override
                                                                                 public void onClick(View v) {
                                                                                     boolean a;
                                                                                     String name = et_pathnamex.getText().toString().trim();
                                                                                     if (name == null || name.length() <= 0) {

                                                                                     } else {
                                                                                         a = Utils.SpecialSymbols(name);
                                                                                         if (a) {
                                                                                             Utils.showToast(getApplicationContext(), "不能包含*&%$#@!等特殊字符");
                                                                                             return;
                                                                                         }
                                                                                         dialog.dismiss();
                                                                                     }

                                                                                     if (stationPathNum == 0)
                                                                                         return;
                                                                                     pathList = mynpu.GetPaths(mapname);
                                                                                     if (pathList == null)
                                                                                         return;
                                                                                     wizrobo_npu.Path[] newPathList = new wizrobo_npu.Path[pathList.length + 1];
                                                                                     for (int i = 0; i < pathList.length; i++) {
                                                                                         newPathList[i] = pathList[i];
                                                                                     }

                                                                                     Pose3D[] poses = new Pose3D[stationPathNum];
                                                                                     for (int i = 0; i < stationPathNum; i++) {
                                                                                         poses[i] = stationPathPose[i];
                                                                                     }
                                                                                     PathInfo newPathInfo = new PathInfo(mapname, name, 0, 0);
                                                                                     wizrobo_npu.Path newpath = new wizrobo_npu.Path(newPathInfo, poses);
                                                                                     newPathList[pathList.length] = newpath;

                                                                                     mynpu.SetPaths(mapname, newPathList);
                                                                                     stationPathNum = 0;
                                                                                     isSettingStationPath = false;
                                                                                     Utils.showToast(getApplicationContext(), "保存成功！");
                                                                                 }
                                                                             }
            );
            listPathStationId.clear();
            imgPathList = mynpu.GetImgPaths(mapname);
            for (int i = 0; i < imgPathList.length; i++) {
                listPathStationId.add(imgPathList[i].info.id);
            }
            adapterPathStationList.notifyDataSetChanged();

        }
    }

    private void TrackPath() {
        if (mynpu.isInited && isTrack) {
            if (imgPathList == null || imgPathList.length == 0)
                return;
            Utils.showToast(getApplicationContext(), "开始执行路径...");
            mynpu.FollowPath(mapname, sp_path_station_list.getSelectedItem().toString());
        }
    }

    private OnClickListener Bt_Reboot_Onclick = new OnClickListener() {
        public void onClick(View v) {
            new AlertDialog.Builder(WizRoboNpu.this, AlertDialog.THEME_DEVICE_DEFAULT_DARK)
                    .setTitle(R.string.str_warming)
                    .setMessage(R.string.str_warnreboot)
                    .setPositiveButton("是",
                            new DialogInterface.OnClickListener() {
                                public void onClick(
                                        DialogInterface dialoginterface,
                                        int i) {
                                    if (isInited) {
                                        mynpu.Reboot();
                                        Utils.showToast(getApplicationContext(), "NPU重启中...");
                                        Intent intent = getIntent();
                                        finish();
                                        startActivity(intent);
                                        updateUIOnce = true;
                                    }
                                }
                            })
                    .setNegativeButton("否",
                            null).show();

        }
    };

    /**
     * @brief :init the ice use the thread to get the udp ip
     * send the message when get the ip success
     * @attention : you must use the thread to get the ip
     */
    private void ConnectNPU() {
        new Thread() {
            public void run() {
                StrictMode.setThreadPolicy(new StrictMode.ThreadPolicy.Builder().detectDiskReads().detectDiskWrites().detectNetwork().penaltyLog().build());
                handler_display_points.post(update_display_points);
                handler_update_ui.post(update_ui);
                displayOnce = false;
                boolean a;
                str_hostIP = udpSendIp.GetIp();
                if (str_hostIP.length() <= 0)
                    return;
                a = mynpu.NpuInit(str_hostIP);
                if (!a) {
                    return;
                }
                isTimeout = false;
            }
        }.start();
    }

    // 更新UI功能
    private void UpdateCmdMotorSpd() {
        if (cmdMotorSpd == null)
            return;
        float[] a = cmdMotorSpd.rpms;
        if (a == null || a.length == 0)
            return;
        tv_cmd_motor_spd.setText("电机指令速度: (" + new DecimalFormat("##0").format(a[0]).toString() + ", " + new DecimalFormat("##0").format(a[1]).toString() + ")[rpm]");
    }

    private void UpdateActMotorSpd() {
        if (actMotorSpd == null)
            return;
        float[] a = actMotorSpd.rpms;
        if (a == null || a.length == 0)
            return;
        tv_act_motor_spd.setText("电机当前速度: (" + new DecimalFormat("##0").format(a[0]).toString() + ", " + new DecimalFormat("##0").format(a[1]).toString() + ")[rpm]");
    }

    private void UpdateCmdVel() {
        if (cmdVel == null)
            return;
        tv_cmd_vel.setText("指令速度: (" + new DecimalFormat("##0.00").format(cmdVel.v_x).toString() + ", " + new DecimalFormat("##0.00").format(cmdVel.v_yaw).toString() + ")[m/s,rad/s]");
    }

    private void UpdateActVel() {
        if (actVel == null)
            return;
        tv_act_vel.setText("当前速度: (" + new DecimalFormat("##0.00").format(actVel.v_x).toString() + ", " + new DecimalFormat("##0.00").format(actVel.v_yaw).toString() + ")[m/s,rad/s]");
    }

    private void UpdateActPose() {
        if (isSlam || isNavi || isTrack) {
            if (isPoseMode) {
                actPoseX = actPose.x;
                actPoseY = actPose.y;
                if (actPose.yaw < 10f && actPose.yaw > -10f)
                    actPoseYaw = actPose.yaw;
                tv_act_posex.setText("当前位置:" + new DecimalFormat("##0.00").format(actPose.x) + ", " + new DecimalFormat("##0.00").format(actPose.y) + ", " + new DecimalFormat("##0.00").format(actPose.yaw));
            }

            if (isImgPoseMode) {
                if (actImgPose == null)
                    return;
                actImgposeU = actImgPose.u;
                actImgposeV = actImgPose.v;
                if (actImgPose.theta < 10f && actImgPose.theta > -10f)
                    actImgposeTheta = actImgPose.theta;
                if (!isToDisplayInitPose && cmdVel.v_x == 0 && cmdVel.v_yaw == 0) {
                    robotInitPoseX = actImgposeU;
                    robotInitPoseY = actImgposeV;
                    isToDisplayInitPose = true;
                }

                if (actPose == null)
                    return;
                actPoseX = actPose.x;
                actPoseY = actPose.y;
                if (actPose.yaw < 10f && actPose.yaw > -10f)
                    actPoseYaw = actPose.yaw;
                tv_act_posex.setText("当前位置:" + new DecimalFormat("##0.00").format(actPose.x) + ", " + new DecimalFormat("##0.00").format(actPose.y) + ", " + new DecimalFormat("##0.00").format(actPose.yaw));
            }
        }
    }

    private void UpdateRobotStatus() {
        if (mynpu.isInited) {
            tv_robot_status.setText("NPU状态:" + naviState.name());
        }
    }

    private void Display() {
        if (toGetImgMap) {
            ReadImgMap();
        }

        if (imgMap == null || bm_background == null) {
            Utils.showToast(getApplicationContext(), "地图刷新中...");
            return;
        }

        Bitmap backgroundbmp_draw = bm_background.copy(Config.ARGB_8888, true);
        Canvas canvas = new Canvas(backgroundbmp_draw);
        Paint paint = new Paint();
        paint.setStyle(Paint.Style.STROKE);
        paint.setAntiAlias(true);      // To solve the zigzag problem.
        canvas.setDrawFilter(new PaintFlagsDrawFilter(0, Paint.ANTI_ALIAS_FLAG | Paint.FILTER_BITMAP_FLAG));
        if (pointIsSelected) {
            paint.setColor(Color.RED);
            paint.setStrokeWidth(1f);
            canvas.drawCircle(setImgposeU, setImgposeV, 4, paint);
        }

        paint.setStyle(Paint.Style.FILL);
        if (isSettingInitialPose) {
            paint.setColor(Color.RED);
            paint.setStrokeWidth(1.0f);
            int desx = initImgposeU + (int) (10f * Math.cos(redarrowYaw));
            int desy = initImgposeV - (int) (10f * Math.sin(redarrowYaw));
            DrawArrow(canvas, paint, initImgposeU, initImgposeV, desx, desy);
        }

        if (isSettingGoalPose || isSettingPathpose || isSettingFreePath) {
            paint.setColor(Color.RED);
            paint.setStrokeWidth(1.0f);
            int desx = setImgposeU + (int) (10f * Math.cos(redarrowYaw));
            int desy = setImgposeV - (int) (10f * Math.sin(redarrowYaw));
            DrawArrow(canvas, paint, setImgposeU, setImgposeV, desx, desy);
        }
        DisplayMapInfoPath(canvas, paint);
        DisplayMapInfoStation(canvas, paint);
        DisplayFreePath(canvas, paint);
        DisplayStationPath(canvas, paint);
        DisplayCmdPath(canvas, paint);
        DisplayFootprintVerticles(canvas, paint);
        DisplayLidarPoints(canvas, paint);
        DisplayInitPose(canvas, paint);

        DisplayActPose(canvas, paint);
        if (pixelMat == null)
            return;
        if (pixelMat.height <= 0 || pixelMat.width <= 0)
            return;
        float compare = 0;
        compare = canvasHeight / pixelMat.height - canvasWidth / pixelMat.width;
        if (compare > 0)
            mapEnlargeLevel = canvasWidth / pixelMat.width;
        else
            mapEnlargeLevel = canvasHeight / pixelMat.height;
        System.out.printf("mapEnlargeLevel = %3.2f, canvasHeight = %3.2f,canvasWidth = %3.2f, compare = %3.2f", mapEnlargeLevel, canvasHeight, canvasWidth, compare);
        if (Float.isNaN(mapEnlargeLevel))
            return;
        backgroundbmp_draw = MapBig(backgroundbmp_draw, mapEnlargeLevel);   // this function need to read more detail ,2017.4.27 modify by jeremy
        iv_background.setImageBitmap(backgroundbmp_draw);
    }

    private void DisplayLidarPoints(Canvas canvas, Paint paint) {
        if (isSlam || isNavi || isTrack) {
            if (toDisplayLidar) {
                if (isPoseMode) {
                    LidarScan lidarscandata = new LidarScan();
                    lidarscandata = lidarScanData;
                    Point3D[] points;
                    points = lidarscandata.points;
                    float[] lidarpointdata = new float[2 * points.length];

                    for (int i = 0; i < points.length; i++) {
                        lidarpointdata[2 * i] = points[i].x;
                        lidarpointdata[2 * i + 1] = points[i].y;
                    }

                    for (int j = 0; j < lidarpointdata.length; j++) {
                        lidarpointdata[j] = (float) ((lidarpointdata[j] - mapInfo.offset.x) / resolution * pixelMat.ratio);
                        lidarpointdata[j + 1] = (float) ((pixelMat.height - (lidarpointdata[j + 1] - mapInfo.offset.y) / resolution * pixelMat.ratio));
                        j++;
                    }

                    paint.setColor(Color.RED);
                    paint.setStrokeWidth(1.0f);
                    canvas.drawPoints(lidarpointdata, paint);
                    Log.d(TAG, "Displaying LidarPoints");
                }

                if (isImgPoseMode) {

                    if (imglidarscandata == null)
                        return;
                    ImgPoint[] points;
                    points = imglidarscandata.points;
                    if (points == null)
                        return;
                    float[] lidarpointdata = new float[2 * points.length];

                    for (int i = 0; i < points.length; i++) {
                        lidarpointdata[2 * i] = (float) points[i].u;
                        lidarpointdata[2 * i + 1] = (float) points[i].v;
                    }

                    for (int j = 0; j < lidarpointdata.length; j++) {
                        lidarpointdata[j] = (float) ((lidarpointdata[j]));
                        lidarpointdata[j + 1] = (float) (((lidarpointdata[j + 1])));
                        j++;
                    }

                    paint.setColor(Color.RED);
                    paint.setStrokeWidth(1.0f);
                    canvas.drawPoints(lidarpointdata, paint);
                    Log.d(TAG, "Displaying LidarPoints");
                }
            }
        }
    }

    private void DisplayFootprintVerticles(Canvas canvas, Paint paint) {
        if (isNavi || isTrack) {
            if (isPoseMode) {
                Point3D[] point3Ds = mynpu.GetFootprintVertices();             //Display the path
                float[] FootprintVerticles = new float[2 * point3Ds.length];
                for (int i = 0; i < point3Ds.length; i++) {
                    FootprintVerticles[2 * i] = point3Ds[i].x;
                    FootprintVerticles[2 * i + 1] = point3Ds[i].y;
                }

                for (int j = 0; j < FootprintVerticles.length; j++) {
                    FootprintVerticles[j] = (float) ((FootprintVerticles[j] - mapInfo.offset.x) / resolution);
                    FootprintVerticles[j + 1] = (float) ((pixelMat.height - (FootprintVerticles[j + 1] - mapInfo.offset.y) / resolution));
                    j++;
                }
                paint.setColor(Color.BLUE);
                paint.setStrokeWidth(2.0f);
                canvas.drawPoints(FootprintVerticles, paint);
                Log.d(TAG, "Displaying FootprintVerticles");

            }

            if (isImgPoseMode) {

                ImgPoint[] imgPoints = FootprintVerticles;
                if (imgPoints == null)
                    return;
                paint.setColor(Color.BLUE);
                paint.setStrokeWidth(1.0f);
                for (int i = 0; i < imgPoints.length - 1; i++) {
                    //paint.setStrokeWidth(1.0f);
                    canvas.drawLine(imgPoints[i].u, imgPoints[i].v, imgPoints[i + 1].u, imgPoints[i + 1].v, paint);
                }
                if (imgPoints.length != 0)
                    canvas.drawLine(imgPoints[0].u, imgPoints[0].v, imgPoints[imgPoints.length - 1].u, imgPoints[imgPoints.length - 1].v, paint);
                Log.d(TAG, "Displaying FootprintVerticles");
            }
        }
    }

    private void DisplayCmdPath(Canvas canvas, Paint paint) {
        //*************************DisplayCmdPath****************************
        if (isNavi || isTrack) {
            if (isPoseMode) {
                path2D = cmdPath;             //Display the path
                Pose3D[] pose;
                pose = path2D.poses;
                float[] pathData = new float[2 * pose.length];
                for (int i = 0; i < pose.length; i++) {
                    pathData[2 * i] = pose[i].x;
                    pathData[2 * i + 1] = pose[i].y;
                }
                for (int j = 0; j < pathData.length; j++) {
                    pathData[j] = (float) ((pathData[j] - mapInfo.offset.x) / resolution);
                    pathData[j + 1] = (float) ((pixelMat.height - (pathData[j + 1] - mapInfo.offset.y) / resolution));
                    j++;
                }
                paint.setColor(Color.RED);
                paint.setStrokeWidth(2.0f);
                canvas.drawPoints(pathData, paint);
                Log.d(TAG, "Displaying CmdPath");
            }

            if (isImgPoseMode) {
                ImgPath imgPath;
                imgPath = cmdImgPath;             //Display the path
                if (imgPath == null)
                    return;
                ImgPose[] pose;
                pose = imgPath.poses;
                if (pose == null)
                    return;
                float[] pathData = new float[2 * pose.length];
                for (int i = 0; i < pose.length; i++) {
                    pathData[2 * i] = pose[i].u;
                    pathData[2 * i + 1] = pose[i].v;
                }

                for (int j = 0; j < pathData.length; j++) {
                    pathData[j] = (float) (pathData[j]);
                    pathData[j + 1] = (float) (pathData[j + 1]);
                    j++;
                }
                paint.setColor(Color.RED);
                paint.setStrokeWidth(2.0f);
                canvas.drawPoints(pathData, paint);
                Log.d(TAG, "Displaying CmdPath");
            }
        }
    }

    private void DisplayMapInfoPath(Canvas canvas, Paint paint) {
        //*************************DisplayMapInfoPath****************************
        if (toDisplayPath && isTrack) {
            if (isPoseMode) {
                Pose3D[] pose;
                pose = pathList[sp_path_station_list.getSelectedItemPosition()].poses;
                float[] pathData = new float[2 * pose.length];
                for (int i = 0; i < pose.length; i++) {
                    pathData[2 * i] = pose[i].x;
                    pathData[2 * i + 1] = pose[i].y;
                }

                for (int j = 0; j < pathData.length; j = j + 2) {
                    pathData[j] = (float) ((pathData[j] - mapInfo.offset.x) / resolution * pixelMat.ratio);
                    pathData[j + 1] = (float) ((pixelMat.height - (pathData[j + 1] - mapInfo.offset.y) / resolution * pixelMat.ratio));
                }
                paint.setColor(Color.GREEN);
                paint.setStrokeWidth(2.0f);
                canvas.drawPoints(pathData, paint);
                Log.d(TAG, "Displaying MapInfoPath");

                //*******Display Lines********
                paint.setColor(Color.BLUE);
                paint.setStrokeWidth(1.5f);
                if (pathData.length <= 2)
                    return;
                else {
                    for (int i = 0; i < pathData.length / 2; i += 2) {
                        canvas.drawLine(pathData[i], pathData[i + 1], pathData[i + 2], pathData[i + 3], paint);
                    }
                }
            }

            if (isImgPoseMode) {
                ImgPose[] pose;
                if (imgPathList == null || imgPathList.length == 0)
                    return;
                for (int k = 0; k < imgPathList.length; k++) {
//                    pose = imgPathList[sp_path_station_list.getSelectedItemPosition()].poses;
                    pose = imgPathList[k].poses;
                    float[] pathData = new float[2 * pose.length];
                    for (int i = 0; i < pose.length; i++) {
                        pathData[2 * i] = pose[i].u;
                        pathData[2 * i + 1] = pose[i].v;
                        paint.setColor(Color.RED);
                        paint.setStrokeWidth(1.0f);
                        int desx = pose[i].u + (int) (10f * Math.cos(pose[i].theta));
                        int desy = pose[i].v - (int) (10f * Math.sin(pose[i].theta));
                        if (i == pose.length - 1)   //dispaly the endpoint arrow
                            DrawArrow(canvas, paint, pose[i].u, pose[i].v, desx, desy);
                    }
                    for (int j = 0; j < pathData.length; j = j + 2) {
                        pathData[j] = (float) ((pathData[j]));
                        pathData[j + 1] = (float) (((pathData[j + 1])));
                    }
                    paint.setColor(Color.GREEN);
                    paint.setStrokeWidth(2.0f);
                    canvas.drawPoints(pathData, paint);
                    Log.d(TAG, "Displaying MapInfoPath");

                    //*******Display Lines********
                    paint.setColor(Color.BLUE);
                    paint.setStrokeWidth(1.0f);
                    if (k == sp_path_station_list.getSelectedItemPosition())
                        paint.setColor(Color.rgb(94, 211, 254));

                    if (pathData.length <= 2)
                        return;
                    else {
                        for (int i = 0; i < pathData.length - 2; i += 2) {
                            canvas.drawLine(pathData[i], pathData[i + 1], pathData[i + 2], pathData[i + 3], paint);
                        }
                    }


                }
            }
        }
    }

    private void DisplayStationPath(Canvas canvas, Paint paint) {
        if (isSettingStationPath) {
            float[] pathData = new float[2 * stationPathNum];

            for (int i = 0; i < stationPathNum; i++) {
                pathData[2 * i] = stationPathPose[i].x;
                pathData[2 * i + 1] = stationPathPose[i].y;
            }
            for (int j = 0; j < pathData.length; j = j + 2) {
                pathData[j] = (float) ((pathData[j] - mapInfo.offset.x) / resolution * pixelMat.ratio);
                pathData[j + 1] = (float) ((pixelMat.height - (pathData[j + 1] - mapInfo.offset.y) / resolution * pixelMat.ratio));

                paint.setColor(Color.RED);
                paint.setStrokeWidth(1.0f);
                int desx = (int) pathData[j] + (int) (10f * Math.cos(stationPathPose[j / 2].yaw));
                int desy = (int) pathData[j + 1] - (int) (10f * Math.sin(stationPathPose[j / 2].yaw));
                DrawArrow(canvas, paint, (int) pathData[j], (int) pathData[j + 1], desx, desy);
            }


            //*******Display points********
            paint.setColor(Color.GREEN);
            paint.setStrokeWidth(3.0f);
            canvas.drawPoints(pathData, paint);
            //*******Display Lines********
            paint.setColor(Color.YELLOW);
            paint.setStrokeWidth(1.5f);
            if (stationPathNum < 2)
                return;
            else {
                for (int i = 0; i < pathData.length / 2; i += 2) {
                    canvas.drawLine(pathData[i], pathData[i + 1], pathData[i + 2], pathData[i + 3], paint);
                }
            }

        }
    }

    private void DisplayFreePath(Canvas canvas, Paint paint) {

        //*************************DisplayFreePath****************************
        if (isSettingPathpose || isSettingFreePath) {
            float[] pathData = new float[2 * pathPoseNum];
            if (isPoseMode) {
                for (int i = 0; i < pathPoseNum; i++) {
                    pathData[2 * i] = pathPose[i].x;
                    pathData[2 * i + 1] = pathPose[i].y;
                }

                for (int j = 0; j < pathData.length; j = j + 2) {
                    pathData[j] = (float) ((pathData[j] - mapInfo.offset.x) / resolution * pixelMat.ratio);
                    pathData[j + 1] = (float) ((pixelMat.height - (pathData[j + 1] - mapInfo.offset.y) / resolution * pixelMat.ratio));
                }
            }

            if (isImgPoseMode) {
                for (int i = 0; i < pathPoseNum; i++) {
                    pathData[2 * i] = imgPathPose[i].u;
                    pathData[2 * i + 1] = imgPathPose[i].v;
                }

                for (int j = 0; j < pathData.length; j = j + 2) {
                    pathData[j] = (float) ((pathData[j]));
                    pathData[j + 1] = (float) (((pathData[j + 1])));
                }
            }
            //*******Display points********
            paint.setColor(Color.GREEN);
            paint.setStrokeWidth(3.0f);
            canvas.drawPoints(pathData, paint);
            Log.d(TAG, "Displaying FreePath");

            //*******Display Lines********
            paint.setColor(Color.BLUE);
            paint.setStrokeWidth(1.0f);
            if (pathPoseNum < 2)
                return;
            else {
                for (int i = 0; i < pathPoseNum - 1; i++) {
                    //paint.setStrokeWidth(1.0f);
                    canvas.drawLine(imgPathPoint[i].u, imgPathPoint[i].v, imgPathPoint[i + 1].u, imgPathPoint[i + 1].v, paint);
                }
            }
        }
    }

    private void DisplayMapInfoStation(Canvas canvas, Paint paint) {
        if (toDisplayStation && isNavi) {
            if (isPoseMode) {
                Pose3D pose;
                pose = stationList[sp_path_station_list.getSelectedItemPosition()].pose;
                int stationx, stationy;
                stationx = (int) ((pose.x - mapInfo.offset.x) / resolution * pixelMat.ratio);
                stationy = (int) ((pixelMat.height - (pose.y - mapInfo.offset.y) / resolution * pixelMat.ratio));
                paint.setColor(Color.RED);
                paint.setStrokeWidth(2.0f);
                canvas.drawCircle(stationx, stationy, 2, paint);
                Log.d(TAG, "Displaying MapInfoStation");
            }
            if (isImgPoseMode) {
                ImgPose pose;
                if (imgStationList == null || imgStationList.length == 0)
                    return;
                pose = imgStationList[sp_path_station_list.getSelectedItemPosition()].pose;
                int stationx, stationy;
                stationx = (int) ((pose.u));
                stationy = (int) (((pose.v)));
                paint.setColor(Color.rgb(0, 249, 208));    //青色
                paint.setStrokeWidth(6.0f);
                canvas.drawCircle(stationx, stationy, 2, paint);
                Log.d(TAG, "Displaying MapInfoStation");

            }
        }
    }

    private void DisplayActPose(Canvas canvas, Paint paint) {
        int arrowx = 0, arrowy = 0;
        ///*****************Display CurrentPose***************************
        if (isPoseMode) {
            if (pixelMat.ratio > 0) {
                arrowx = (int) ((actPoseX - mapInfo.offset.x) / resolution * pixelMat.ratio);
                arrowy = (int) ((pixelMat.height - (actPoseY - mapInfo.offset.y) / resolution * pixelMat.ratio));
            } else {
                arrowx = (int) ((actPoseY - mapInfo.offset.y) / resolution * (pixelMat.ratio));
                arrowy = (int) ((pixelMat.height - (actPoseX - mapInfo.offset.x) / resolution * (-pixelMat.ratio)));
            }
        }

        ///********************DisplayImgPose*********************************
        if (isImgPoseMode) {
            if (pixelMat == null)
                return;


            if (pixelMat.ratio > 0) {
                arrowx = (int) ((actImgposeU));
                arrowy = (int) (((actImgposeV)));
            } else {   //System.out.println("the ratio is :"+pixelMat.ratio);
                arrowx = (int) (actImgposeV);
                arrowy = (int) (((actImgposeU) * (-1)));
            }
        }

        paint.setColor(Color.BLUE);
        canvas.drawCircle(arrowx, arrowy, 4, paint);
        paint.setColor(Color.BLUE);
        paint.setStrokeWidth(1.5f);
        float rad = 0;
        if (isPoseMode)
            rad = actPoseYaw;
        if (isImgPoseMode)
            rad = actImgposeTheta;
        if (pixelMat.ratio > 0) {
            int desx = arrowx + (int) (10f * Math.cos(rad));
            int desy = arrowy - (int) (10f * Math.sin(rad));
            DrawAL(canvas, paint, arrowx, arrowy, desx, desy);
        } else {
            int desx = arrowx + (int) (10f * Math.cos(rad + 1.57));
            int desy = arrowy - (int) (10f * Math.sin(rad + 1.57));
            DrawAL(canvas, paint, arrowx, arrowy, desx, desy);
        }
        Log.d(TAG, "Displaying ActPose");
    }

    private void DisplayInitPose(Canvas canvas, Paint paint) {
        if (isTrack || isNavi) {
            if (isToDisplayInitPose) {
                paint.setStrokeWidth(2f);
                paint.setColor(Color.YELLOW);
                canvas.drawCircle(robotInitPoseX, robotInitPoseY, 5, paint);
            }
        }
    }

    private static Bitmap MapBig(Bitmap bitmap, float scare) {
        Matrix matrix = new Matrix();
        matrix.postScale(scare, scare);
        Bitmap resizeBmp = Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(), bitmap.getHeight(), matrix, true);
        return resizeBmp;
    }

    Bitmap Convert(Bitmap a, int width, int height) {
        if (a == null)
            return null;
        int w = a.getWidth();
        int h = a.getHeight();
        Bitmap newb = Bitmap.createBitmap(w, h, Config.ARGB_8888);
        Canvas cv = new Canvas(newb);
        Matrix m = new Matrix();
        Bitmap new2 = Bitmap.createBitmap(a, 0, 0, w, h, m, true);

        cv.drawBitmap(new2, new Rect(0, 0, new2.getWidth(), new2.getHeight()), new Rect(0, 0, w, h), null);
        return newb;
    }

    private Bitmap GetBitmapFromPgm(byte[] decodedString, int width, int height, int dataOffset) {
        if (width <= 1 || height <= 1) {
            return null;
        }
        int[] pixels = new int[width * height];
        int i = 0;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int gray = 0xff - decodedString[dataOffset + i] & 0xff;
                pixels[i] = 0xff000000 | gray << 16 | gray << 8 | gray;
                // pixels[i] = decodedString[i] & 0xff;
                i++;
            }
        }

        if (width <= 0 || height <= 0) {
            return null;
        }


        Bitmap pgm = Bitmap.createBitmap(pixels, width, height, Config.ARGB_8888);
        return pgm;
    }

    private double[] RotateVec(int px, int py, double ang, boolean isChLen, double newLen) {
        double mathstr[] = new double[2];
        double vx = px * Math.cos(ang) - py * Math.sin(ang);
        double vy = px * Math.sin(ang) + py * Math.cos(ang);
        if (isChLen) {
            double d = Math.sqrt(vx * vx + vy * vy);
            vx = vx / d * newLen;
            vy = vy / d * newLen;
            mathstr[0] = vx;
            mathstr[1] = vy;
        }
        return mathstr;
    }

    public Path DrawAL(Canvas canvas, Paint paint, int sx, int sy, int ex, int ey) {
        double H = 4;
        double L = 2;
        int x3 = 0;
        int y3 = 0;
        int x4 = 0;
        int y4 = 0;
        double awrad = Math.atan(L / H);
        double arraow_len = Math.sqrt(L * L + H * H);
        double[] arrXY_1 = RotateVec(ex - sx, ey - sy, awrad, true, arraow_len);
        double[] arrXY_2 = RotateVec(ex - sx, ey - sy, -awrad, true, arraow_len);
        double x_3 = ex - arrXY_1[0];
        double y_3 = ey - arrXY_1[1];
        double x_4 = ex - arrXY_2[0];
        double y_4 = ey - arrXY_2[1];
        Double X3 = new Double(x_3);
        x3 = X3.intValue();
        Double Y3 = new Double(y_3);
        y3 = Y3.intValue();
        Double X4 = new Double(x_4);
        x4 = X4.intValue();
        Double Y4 = new Double(y_4);
        y4 = Y4.intValue();

        canvas.drawLine(sx, sy, ex, ey, paint);
        Path triangle = new Path();
        triangle.moveTo(ex, ey);
        triangle.lineTo(x3, y3);
        triangle.lineTo(x4, y4);
        triangle.close();
        return triangle;
    }

    public void DrawArrow(Canvas canvas, Paint paint, int sx, int sy, int ex, int ey) {
        canvas.drawPath(DrawAL(canvas, paint, sx, sy, ex, ey), paint); //draw arrow head
    }

    // Add draw scale
    private float Distance(MotionEvent event) {
        try {
            float dx = event.getX(1) - event.getX(0);
            float dy = event.getY(1) - event.getY(0);
            return (float) Math.sqrt(dx * dx + dy * dy);
        } catch (IllegalArgumentException e) {
            e.printStackTrace();
        }
        return -1f;
    }

    private void ReadImgMap() {
        isReadThumbnail = false;
        if (imgMap == null)
            return;
        mapInfo = imgMap.info;
        pixelMat = imgMap.mat;
        if (imgMap == null || pixelMat == null || mapInfo == null)
            return;
        Log.d(TAG, "pixelMat.ratio" + pixelMat.ratio);
        if (pixelMat.height == 0 || pixelMat.width == 0 || mapInfo.resolution == 0 || imgMap.mat == null)
            return;
        resolution = (float) mapInfo.resolution;
        DecimalFormat fnum = new DecimalFormat("##0.00");
        tv_map_info.setText("地图信息:" + Integer.toString(pixelMat.width) + ", " + Integer.toString(pixelMat.height) + ", " + fnum.format(mapInfo.resolution));
        bm_background = GetBitmapFromPgm(pixelMat.data, pixelMat.width, pixelMat.height, 0);
        bm_background = Convert(bm_background, pixelMat.width, pixelMat.height);
    }

    public void ReadThumbnail() {
        isReadThumbnail = true;
        pixelMat = mapInfoList[sp_map_list.getSelectedItemPosition()].thumbnail;
        if (pixelMat.width <= 0 || pixelMat.height <= 0)
            return;
        mapInfo = mapInfoList[sp_map_list.getSelectedItemPosition()];
        resolution = mapInfo.resolution;
        Log.d(TAG, "the map resolution is:" + resolution);
        if (pixelMat.width < 0 || pixelMat.height < 0)
            return;
        tv_map_info.setText("MapInfo:" + Integer.toString(pixelMat.width) + ", " + Integer.toString(pixelMat.height) + ", " + Float.toString(mapInfo.resolution));
        bm_background = GetBitmapFromPgm(pixelMat.data, pixelMat.width, pixelMat.height, 0);
        if (bm_background == null)
            return;
        bm_background = Convert(bm_background, pixelMat.width, pixelMat.height);
    }

    //@Override
    public void onJoystickMoved(float xPercent, float yPercent, int id) {
        switch (id) {
            case R.id.joystickLeft:
                linScale = xPercent;
                angScale = -yPercent;
                Log.d(TAG, "linScale:  " + linScale + "   angScale" + angScale);
                if (isSettingInitialPose || isSettingGoalPose) {
                    if (linScale == 0 && angScale == 0)
                        return;
                    redarrowYaw = (float) (Math.atan2(linScale, -angScale));
                    initImgposeTheta = redarrowYaw;
                    setImgposeTheta = redarrowYaw;
                    if (pathPoseNum == 0)
                        return;
                    if (isSettingFreePath)
                        imgPathPose[pathPoseNum - 1].theta = redarrowYaw;
                } else if (isSettingPathpose) {
                    if (linScale == 0 && angScale == 0)
                        return;
                    redarrowYaw = (float) (Math.atan2(linScale, -angScale));
                    if (pathPoseNum == 0)
                        return;
                    imgPathPose[pathPoseNum - 1].theta = redarrowYaw;
                } else if (toModifyPath) {
                    if (pointIsSelected) {
                        if (imgPathList == null)
                            return;
                        if (linScale == 0 && angScale == 0)
                            toMove = true;
                        if (toMove) {
                            for (int i = 0; i < imgPathList.length; i++) {
                                for (int j = 0; j < imgPathList[i].poses.length; j++) {
                                    if (Math.abs(setImgposeU - imgPathList[i].poses[j].u) < 10 && Math.abs(setImgposeV - imgPathList[i].poses[j].v) < 10) {
                                        if (angScale < -0.9 && Math.abs(linScale) < 0.2) {
                                            imgPathList[i].poses[j].u = imgPathList[i].poses[j].u + 1;
                                            setImgposeU = imgPathList[i].poses[j].u;
                                            toMove = false;
                                        }

                                        if (angScale > 0.9 && Math.abs(linScale) < 0.2) {
                                            imgPathList[i].poses[j].u = imgPathList[i].poses[j].u - 1;
                                            setImgposeU = imgPathList[i].poses[j].u;
                                            toMove = false;
                                        }

                                        if (linScale > 0.9 && Math.abs(angScale) < 0.2) {
                                            imgPathList[i].poses[j].v = imgPathList[i].poses[j].v - 1;
                                            setImgposeV = imgPathList[i].poses[j].v;
                                            toMove = false;
                                        }

                                        if (linScale < -0.9 && Math.abs(angScale) < 0.2) {
                                            imgPathList[i].poses[j].v = imgPathList[i].poses[j].v + 1;
                                            setImgposeV = imgPathList[i].poses[j].v;
                                            toMove = false;
                                        }
                                    }
                                }
                            }
                        }
                    }
                } else if (toModifyPathYaw) {
                    if (pointIsSelected) {
                        if (imgPathList == null)
                            return;

                        if (linScale == 0 && angScale == 0)
                            return;
                        for (int i = 0; i < imgPathList.length; i++) {
                            for (int j = 0; j < imgPathList[i].poses.length; j++) {
                                if (Math.abs(setImgposeU - imgPathList[i].poses[j].u) < 10 && Math.abs(setImgposeV - imgPathList[i].poses[j].v) < 10) {

                                    imgPathList[i].poses[j].theta = (float) (Math.atan2(linScale, -angScale));
                                }
                            }
                        }
                    }
                } else {
                    isJoystickTriggered = true;

                    if (Math.abs(linScale) < 0.3)
                        linScale = 0;
                    if (Math.abs(angScale) < 0.3)
                        angScale = 0;
                    if (linScale == 0 && angScale == 0) {
                        isJoystickTriggered = false;
                    }
                    Log.d(TAG, "linScale is:" + linScale + " angScale is:" + angScale);
                }

                break;
            default:
                break;
        }
    }

    /**
     * 检测信号强度
     */
    private void GetWifiStrength() {
        String wserviceName = Context.WIFI_SERVICE;
        WifiManager wm = (WifiManager) getSystemService(wserviceName);
        WifiInfo info = wm.getConnectionInfo();
        wifiStrength = info.getRssi();
        tv_wifi_strength.setTextColor(Color.rgb(255, 116, 0));
        tv_wifi_strength.setText("   信号" + Integer.toString(wifiStrength) + " " + "良好");
        if (wifiStrength < -69) {
            toStopGetImgMap = true;
            tv_wifi_strength.setTextColor(Color.rgb(207, 42, 48));
            tv_wifi_strength.setText("   信号" + Integer.toString(wifiStrength) + " " + "弱");
            Utils.showToast(getApplicationContext(), "wifi信号弱");
        } else {
            if (isSlam == true && toStopGetImgMap == true) {
                toStopGetImgMap = false;
            }
        }
    }

    @Override
    public boolean onKeyDown(int keyCode, KeyEvent event) {
        if (keyCode == KeyEvent.KEYCODE_BACK) {
            ShowDialog.showExitDialog(WizRoboNpu.this);
        }
        return false;
    }

    public void ExceptionAlert(Exception e) {
        new AlertDialog.Builder(WizRoboNpu.this, AlertDialog.THEME_DEVICE_DEFAULT_DARK)
                .setTitle(R.string.str_warming)
                .setMessage("异常：" + e.toString())
                .setPositiveButton(R.string.str_ok,
                        new DialogInterface.OnClickListener() {
                            public void onClick(
                                    DialogInterface dialoginterface,
                                    int i) {
                                if (mynpu.isInited) {
                                    Intent intent = getIntent();
                                    finish();
                                    startActivity(intent);
                                }
                            }
                        })
                .setNegativeButton(R.string.str_no, null).show();
    }
}


