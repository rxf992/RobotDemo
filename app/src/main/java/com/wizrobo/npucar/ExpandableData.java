package com.wizrobo.npucar;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Created by dongjiang on 2017/8/9.
 */

public class ExpandableData {
    public static HashMap<String, List<String>> getFuctionData()
    {
        HashMap<String, List<String>> expandableListDetail = new HashMap<String, List<String>>();

        List<String> function = new ArrayList<String>();
        function.add("开始导航");
        function.add("开始建图");
        expandableListDetail.put("功能选择", function);
        return expandableListDetail;
    }

    public static HashMap<String, List<String>> getOperateData()
    {
        HashMap<String, List<String>> expandableListDetail = new HashMap<String, List<String>>();




        if(WizRoboNpu.isTrack) {


            List<String> excute = new ArrayList<String>();
            excute.add("单条路径执行");

            List<String> addpath = new ArrayList<String>();
            addpath.add("自动添加路径");
            addpath.add("设为路径点");
            addpath.add("保存站点路径");

            List<String> modify = new ArrayList<String>();
            modify.add("修改XY值");
            modify.add("修改角度");
            modify.add("删除路径");

            expandableListDetail.put("添加路径", addpath);
            expandableListDetail.put("编辑路径", modify);
            expandableListDetail.put("执行路径", excute);
            //expandableListDetail.put("设置", setting);

        }

        if(WizRoboNpu.isNavi)
        {
            List<String> operate = new ArrayList<String>();
            operate.add("添加站点");
            operate.add("删除站点");
            operate.add("到达站点");
            expandableListDetail.put("站点处理", operate);
        }


        return expandableListDetail;
    }

    public static HashMap<String, List<String>> getSettingData()
    {
        HashMap<String, List<String>> expandableListDetail = new HashMap<String, List<String>>();
        List<String> setting = new ArrayList<String>();
        if(WizRoboNpu.isNavi) {
            setting.add("设置初始位");
            setting.add("设置目标点");
        }

        if(WizRoboNpu.isTrack)
        {
            setting.add("设置初始位");
            setting.add("设置自由路径");
        }
        expandableListDetail.put("设置", setting);
        return expandableListDetail;
    }
}
