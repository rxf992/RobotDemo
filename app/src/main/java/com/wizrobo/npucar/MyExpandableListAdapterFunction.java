package com.wizrobo.npucar;

/**
 * Created by wizrobo on 7/11/17.
 */

import android.content.Context;
import android.graphics.Color;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseExpandableListAdapter;
import android.widget.TextView;

import java.util.HashMap;
import java.util.List;

public class MyExpandableListAdapterFunction extends BaseExpandableListAdapter
{
    private Context mContext;
    private List<String> expandableListTitle;
    private HashMap<String, List<String>> expandableListDetail;

    public MyExpandableListAdapterFunction(List<String> expandableListTitle, HashMap<String,List<String>> expandableListDetail, Context mContext)
    {
        this.expandableListDetail = expandableListDetail;
        this.expandableListTitle = expandableListTitle;
        this.mContext = mContext;
    }

    @Override
    public int getGroupCount()//分组数
    {
        return this.expandableListTitle.size();
    }
    @Override
    public int getChildrenCount(int groupPosition)//分组内的item数
    {
        return this.expandableListDetail.get(expandableListTitle.get(groupPosition)).size();
    }
    @Override
    public Object getGroup(int groupPosition)//获取分组数据
    {
        return this.expandableListTitle.get(groupPosition);
    }
    @Override
    public Object getChild(int groupPosition, int childPosition)//获取第几分组第几个item的数据
    {
        return this.expandableListDetail.get(this.expandableListTitle.get(groupPosition)).get(childPosition);
    }
    @Override
    public long getGroupId(int groupPosition)
    {
        return groupPosition;
    }
    @Override
    public long getChildId(int groupPosition, int childPosition)
    {
        return childPosition;
    }
    @Override
    public boolean hasStableIds()
    {
        return false;
    }
    @Override
    public View getGroupView(int groupPosition, boolean isExpanded,
                             View convertView, ViewGroup parent)
    {
        String data = this.expandableListTitle.get(groupPosition);
        if(convertView == null)
        {
            convertView = View.inflate(mContext,R.layout.list_item, null);
        }
        TextView tv = (TextView) convertView.findViewById(R.id.expandedListItem);
        tv.setText(data);
        return convertView;
    }
    @Override
    public View getChildView(int groupPosition, int childPosition,
                             boolean isLastChild, View convertView, ViewGroup parent)
    {
//		String data = this.expandableListDetail.get(this.expandableListTitle.get(groupPosition)).get(childPosition);
        String data = (String) this.getChild(groupPosition, childPosition);

        if(convertView == null)
        {
            convertView = View.inflate(mContext, R.layout.list_item, null);
        }
        TextView tv = (TextView) convertView.findViewById(R.id.expandedListItem);
        tv.setTextColor(Color.rgb(90, 90, 90));
        tv.setTextSize(10);


            if (groupPosition == 0 && childPosition == 0 ) {
                WizRoboNpu.track_navi_parent=parent;
                WizRoboNpu.track_navi_view=convertView;
                data = WizRoboNpu.strNaviTrack;
                if(WizRoboNpu.isNavi||WizRoboNpu.isTrack)
                    tv.setTextColor(Color.rgb(207, 42, 48));
                else
                    tv.setTextColor(Color.rgb(90, 90, 90));
        }

        if (groupPosition == 0 && childPosition == 1 ) {
            WizRoboNpu.slam_parent=parent;
            WizRoboNpu.slam_view=convertView;
            data = WizRoboNpu.strSlam;
            if (WizRoboNpu.isSlam)
                tv.setTextColor(Color.rgb(207, 42, 48));
            else
                tv.setTextColor(Color.rgb(90, 90, 90));
        }
        tv.setText(data);
        return convertView;
    }
    @Override
    public boolean isChildSelectable(int groupPosition, int childPosition)
    {
        return true;
    }
}
