package com.wizrobo.npucar;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.support.v7.app.AppCompatActivity;

/**
 * Created by dongjiang on 2017/8/10.
 */

public class ShowDialog {
    public static void showConnectFailed(Context context){
        new AlertDialog.Builder(context, AlertDialog.THEME_DEVICE_DEFAULT_DARK)
                .setTitle("NPU连接失败")
                .setMessage("请先检查NPU及网络连接是否正常，再长按屏幕中间重新连接！")
                .setPositiveButton(R.string.str_ok,
                        null)
                .setNegativeButton(R.string.str_no,
                        null).show();

    }

    public static void showExceptionDialog(Context context,Exception e){
        new AlertDialog.Builder(context, AlertDialog.THEME_DEVICE_DEFAULT_DARK)
                .setTitle(R.string.str_warming)
                .setMessage("异常：" + e.toString())
                .setPositiveButton(R.string.str_ok,
                        new DialogInterface.OnClickListener() {
                            public void onClick(
                                    DialogInterface dialoginterface,
                                    int i) {
//                                if (mynpu.isInited) {
//                                    Intent intent = getIntent();
//                                    finish();
//                                    startActivity(intent);
//                                }
                            }
                        })
                .setNegativeButton(R.string.str_no, null).show();
    }

    public static void showExitDialog(final Context context){
        new AlertDialog.Builder(context, AlertDialog.THEME_DEVICE_DEFAULT_DARK)
                .setTitle("系统提示")
                .setMessage("确定要退出APP吗")
                .setPositiveButton("确定",
                        new DialogInterface.OnClickListener() {
                            public void onClick(
                                    DialogInterface dialoginterface,
                                    int i) {
                                android.os.Process.killProcess(android.os.Process.myPid());
                                System.exit(0);
                                ((AppCompatActivity)context).finish();
                            }
                        })
                .setNegativeButton("取消",
                        null).show();
    }

}
