#include<ros/ros.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
//#include <tf/transform_broadcaster.h>
#include <iterator>


class Pepper_astar {

public:
    int grid_value[14][12];
    std::vector<int> unwalkable;
    std::vector<int> G;
    std::vector<int> H;
    std::vector<int> F;
    std::vector<int> parent;
    std::vector<int> close_list;
    std::vector<int> open_list;
    std::vector<int> path;
    int index_start;
    int index_end;
    int next_index;
    int step_number;
    bool unwalkable_ready;
    bool path_ready;
    bool old_pose_ready;
//    bool tf_ready;
    double error, old_x, old_y, current_x, current_y;

    ros::NodeHandle n;
    ros::Subscriber map_sub;
    ros::Publisher vel_pub;
    ros::Subscriber odom_sub;

    Pepper_astar(ros::NodeHandle n_);


    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void get_H(int);
    std::vector<int> get_F(std::vector<int>, std::vector<int>);
    void get_initial_opG(int);
    void get_opG(int);
    void get_next_index(void);
    void astar(void);
    void get_path(void);


};

Pepper_astar::Pepper_astar(ros::NodeHandle n_) :
    n(n_), grid_value{0},G(168,0),H(168,0),F(168,0),parent(168,0),close_list(0),open_list(0),unwalkable(0),path(0)
{

    index_start=12*10+4;
    index_end=12*4+5;
//    index_start=9*12+4;
//    index_end=4*12+4;
    next_index=0;
    step_number=0;
    unwalkable_ready=false;
    path_ready=false;
    old_pose_ready=false;
//    tf_ready=false;
    error=0;
    old_x=0;
    old_y=0;
    current_x=0;
    current_y=0;

    map_sub=n.subscribe("/map", 1000, &Pepper_astar::mapCallback, this);
    vel_pub=n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    odom_sub=n.subscribe("/pepper_robot/odom", 1000, &Pepper_astar::odomCallback, this);


}

void Pepper_astar::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    if(unwalkable_ready == false)
    {
        int height =msg->info.height;
        int width =msg->info.width;

        for (int i=0; i< height; i++)
        {
            for(int j=0;j< width; j++)
            {

                if (msg->data.at(width*i+j) == 100)
                {
                    grid_value[(280-i)/20][j/20]++;

                }

            }

        }

        for(int i=0;i<14;i++)
        {
            for(int j=0;j<12;j++)
            {
                //grid[i][j]=false;

                if (grid_value[i][j]>30)
                {
                    //grid[i][j]=true;
                    unwalkable.push_back(12*i+j);
                }
            }
        }

        if(unwalkable.size()==61)
        {
            unwalkable_ready= true;
        }
    }
    else
    {
        ROS_INFO("unwalkable is ready");
    }


}

void Pepper_astar::get_H(int index)
{
    int a=index/12;
    int b=index%12;

    for (int i=0; i<H.size(); i++)
    {
        int m=i/12;
        int n=i%12;
        H.at(i)=10*(std::abs(m-a)+std::abs(n-b));
    }
}

std::vector<int> Pepper_astar::get_F(std::vector<int> v1, std::vector<int> v2)
{
    std::vector<int> v3(v1.size());
    for(int i=0;i<v1.size();i++)
    {
        v3.at(i)=v1.at(i)+v2.at(i);
    }
    return v3;
}

void Pepper_astar::get_initial_opG(int index_start)
{
    parent.at(index_start-13)=index_start;
    parent.at(index_start-12)=index_start;
    parent.at(index_start-11)=index_start;
    parent.at(index_start-1)=index_start;
    parent.at(index_start+1)=index_start;
    parent.at(index_start+11)=index_start;
    parent.at(index_start+12)=index_start;
    parent.at(index_start+13)=index_start;
    G.at(index_start+1)=10;
    G.at(index_start-1)=10;
    G.at(index_start+12)=10;
    G.at(index_start-12)=10;
    G.at(index_start+11)=14;
    G.at(index_start-11)=14;
    G.at(index_start+13)=14;
    G.at(index_start-13)=14;
    open_list.push_back(index_start-13);
    open_list.push_back(index_start-12);
    open_list.push_back(index_start-11);
    open_list.push_back(index_start-1);
    open_list.push_back(index_start+1);
    open_list.push_back(index_start+11);
    open_list.push_back(index_start+12);
    open_list.push_back(index_start+13);
}

void Pepper_astar::get_opG(int index_now)
{
    if ( std::find(std::begin(close_list), std::end(close_list), index_now-13) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now-13) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now-13) == std::end(open_list))
        {
            open_list.push_back(index_now-13);
            parent.at(index_now-13)=index_now;
            G.at(index_now-13)=G.at(index_now)+14;
        }
        else
        {
            if(G.at(index_now-13)>G.at(index_now)+14)
            {ROS_WARN_STREAM(__LINE__);
                parent.at(index_now-13)=index_now;
                G.at(index_now-13)=G.at(index_now)+14;
            }

        }
    }
    if ( std::find(std::begin(close_list), std::end(close_list), index_now-12) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now-12) == std::end(unwalkable))
    {
        if (std::find(std::begin(open_list), std::end(open_list), index_now-12) == std::end(open_list))
        {
            open_list.push_back(index_now-12);
            parent.at(index_now-12)=index_now;
            G.at(index_now-12)=G.at(index_now)+10;
        }
        else
        {
            if(G.at(index_now-12)>G.at(index_now)+10)
            {ROS_WARN_STREAM(__LINE__);
                parent.at(index_now-12)=index_now;
                G.at(index_now-12)=G.at(index_now)+10;

            }

        }

    }
    if ( std::find(std::begin(close_list), std::end(close_list), index_now-11) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now-11) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now-11) == std::end(open_list))
        {
            open_list.push_back(index_now-11);
            parent.at(index_now-11)=index_now;
            G.at(index_now-11)=G.at(index_now)+14;
        }
        else
        {
            if(G.at(index_now-11)>G.at(index_now)+14)
            {ROS_WARN_STREAM(__LINE__);
                parent.at(index_now-11)=index_now;
                G.at(index_now-11)=G.at(index_now)+14;

            }

        }
    }
    if ( std::find(std::begin(close_list), std::end(close_list), index_now-1) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now-1) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now-1) == std::end(open_list))
        {
            open_list.push_back(index_now-1);
            parent.at(index_now-1)=index_now;
            G.at(index_now-1)=G.at(index_now)+10;
        }
        else
        {
            if(G.at(index_now-1)>G.at(index_now)+10)
            {ROS_WARN_STREAM(__LINE__);
                parent.at(index_now-1)=index_now;
                G.at(index_now-1)=G.at(index_now)+10;

            }

        }
    }
    if ( std::find(std::begin(close_list), std::end(close_list), index_now+1) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now+1) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now+1) == std::end(open_list))
        {
            open_list.push_back(index_now+1);
            parent.at(index_now+1)=index_now;
            G.at(index_now+1)=G.at(index_now)+10;
        }
        else
        {
            if(G.at(index_now+1)>G.at(index_now)+10)
            {ROS_WARN_STREAM(__LINE__);
                parent.at(index_now+1)=index_now;
                G.at(index_now+1)=G.at(index_now)+10;

            }

        }
    }
    if (std::find(std::begin(close_list), std::end(close_list), index_now+11) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now+11) == std::end(unwalkable))
    {

        if(std::find(std::begin(open_list), std::end(open_list), index_now+11) == std::end(open_list))
        {
            open_list.push_back(index_now+11);
            parent.at(index_now+11)=index_now;
            G.at(index_now+11)=G.at(index_now)+14;
        }
        else
        {
            if(G.at(index_now+11)>G.at(index_now)+14)
            {ROS_WARN_STREAM(__LINE__);
                parent.at(index_now+11)=index_now;
                G.at(index_now+11)=G.at(index_now)+14;
            }

        }

    }
    if (std::find(std::begin(close_list), std::end(close_list), index_now+12) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now+12) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now+12) == std::end(open_list))
        {
            open_list.push_back(index_now+12);
            parent.at(index_now+12)=index_now;
            G.at(index_now+12)=G.at(index_now)+10;
        }
        else
        {
            if(G.at(index_now+12)>G.at(index_now)+10)
            {ROS_WARN_STREAM(__LINE__);
                parent.at(index_now+12)=index_now;
                G.at(index_now+12)=G.at(index_now)+10;

            }

        }
    }
    if (std::find(std::begin(close_list), std::end(close_list), index_now+13) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now+13) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now+13) == std::end(open_list))
        {
            open_list.push_back(index_now+13);
            parent.at(index_now+13)=index_now;
            G.at(index_now+13)=G.at(index_now)+14;
        }
        else
        {
            if(G.at(index_now+13)>G.at(index_now)+14)
            {ROS_WARN_STREAM(__LINE__);
                parent.at(index_now+13)=index_now;
                G.at(index_now+13)=G.at(index_now)+14;
            }

        }
    }
}

void Pepper_astar::get_next_index(void)
{
    next_index=open_list.at(0);
    for(int i=0;i<open_list.size()-1;i++)
    {

        if(F.at(open_list.at(i+1))<F.at(next_index))
        {
            next_index=open_list.at(i+1);
        }

    }

}

void Pepper_astar::astar(void)
{
    close_list.push_back(index_start);
    get_H(index_end);
    get_initial_opG(index_start);
    F=get_F(G,H);
    get_next_index();

    std::vector<int>::iterator position = std::find(open_list.begin(), open_list.end(), index_end);

    int count=0;

    while ((position != open_list.end())==false)//while index_end is not in the open_list

    {
        close_list.push_back(next_index);
        //remove next_index from the open_list
        std::vector<int>::iterator position2 = std::find(open_list.begin(), open_list.end(), next_index);
        if (position2 != open_list.end())
        {
            open_list.erase(position2);
        }

        get_opG(next_index);
        F=get_F(G,H);
        get_next_index();
        position = std::find(open_list.begin(), open_list.end(), index_end);
        count++;
    }
    ROS_INFO_STREAM("count is " <<count);

}

void Pepper_astar::get_path(void)
{
    path.push_back(index_end);
    int my_parent=parent.at(index_end);
    while(my_parent != index_start)
    {
        path.push_back(my_parent);
        my_parent=parent.at(my_parent);
    }
    path.push_back(index_start);
    std::reverse(path.begin(),path.end());

}

void Pepper_astar::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(path_ready == true)
    {
        if(old_pose_ready == false)
        {
            old_x=msg->pose.pose.position.x;
            old_y=msg->pose.pose.position.y;
            old_pose_ready = true;
            ROS_INFO_STREAM("old_x in callback"<<old_x);
        }
        current_x=msg->pose.pose.position.x;
        current_y=msg->pose.pose.position.y;

    }
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"pepper_astar");

    ros::NodeHandle n;
    Pepper_astar pa(n);
    ros::Rate loop_rate(60);

    while(ros::ok())
    {
        ros::spinOnce();

        if(pa.unwalkable_ready == true && pa.path_ready == false)
        {
            pa.astar();
            pa.get_path();
            ROS_INFO_STREAM("path size is "<<pa.path.size());
            for(int i=0;i<pa.path.size();i++)
            {
                ROS_INFO_STREAM(i<<"th parent"<<pa.path.at(i));
            }

            pa.path_ready=true;
        }

        if(pa.path_ready == true)
        {

            int current_point=pa.path.at(pa.step_number);
            int next_point=pa.path.at(pa.step_number+1);
            ROS_WARN_STREAM(__LINE__);
            ROS_INFO_STREAM("current_point "<<current_point);
            ROS_INFO_STREAM("next_point "<<next_point);
            geometry_msgs::Twist velocity;

            if (next_point == (current_point-13))
            {   ROS_WARN_STREAM(__LINE__);
                double distance=std::sqrt(2)*0.4;
                ROS_INFO_STREAM("distance "<<distance);
                if(pa.error <= distance)
                {
                    velocity.linear.x=0.06;
                    velocity.linear.y=0.05;
                    pa.vel_pub.publish(velocity);

                    ROS_INFO_STREAM("current_x "<<pa.current_x);
                    pa.error =std::sqrt(std::pow((pa.current_x-pa.old_x),2)+std::pow((pa.current_y-pa.old_y),2));
                    ROS_INFO_STREAM("error "<<pa.error);
                }
                else
                {
                    velocity.linear.x=0;
                    velocity.linear.y=0;
                    pa.vel_pub.publish(velocity);
                    pa.step_number++;
                    pa.error=0;
                    pa.old_x = pa.current_x;
                    pa.old_y = pa.current_y;
                    ROS_INFO_STREAM("current_x "<<pa.current_x);
                    ROS_INFO_STREAM("step_number is "<<pa.step_number);
                    ROS_INFO_STREAM("old_x "<<pa.old_x);
                    ROS_INFO_STREAM("error "<<pa.error);

                }
            }

            else if (next_point == (current_point-12))
            {   ROS_WARN_STREAM(__LINE__);
                double distance=0.4;
                ROS_INFO_STREAM("distance "<<distance);
                if(pa.error <= distance)
                {
                    velocity.linear.x=0.06;
                    velocity.linear.y=0;
                    pa.vel_pub.publish(velocity);

                    //ROS_INFO_STREAM("current_x "<<pa.current_x);
                    pa.error =std::sqrt(std::pow((pa.current_x-pa.old_x),2)+std::pow((pa.current_y-pa.old_y),2));
                    //ROS_INFO_STREAM("error "<<pa.error);
                }
                else
                {
                    velocity.linear.x=0;
                    velocity.linear.y=0;
                    pa.vel_pub.publish(velocity);
                    pa.step_number++;
                    pa.error=0;
                    pa.old_x = pa.current_x;
                    pa.old_y = pa.current_y;
                    ROS_INFO_STREAM("error "<<pa.error);

                }

            }
            else if (next_point == (current_point-11))
            {   ROS_WARN_STREAM(__LINE__);
                double distance=std::sqrt(2)*0.4;
                ROS_INFO_STREAM("distance "<<distance);
                if(pa.error <= distance)
                {
                    velocity.linear.x=0.06;
                    velocity.linear.y=-0.05;
                    pa.vel_pub.publish(velocity);

                    //ROS_INFO_STREAM("current_x "<<pa.current_x);
                    pa.error =std::sqrt(std::pow((pa.current_x-pa.old_x),2)+std::pow((pa.current_y-pa.old_y),2));
                    //ROS_INFO_STREAM("error "<<pa.error);
                }
                else
                {
                    velocity.linear.x=0;
                    velocity.linear.y=0;
                    pa.vel_pub.publish(velocity);
                    pa.step_number++;
                    pa.error=0;
                    pa.old_x = pa.current_x;
                    pa.old_y = pa.current_y;
                    ROS_INFO_STREAM("error "<<pa.error);
                }

            }
            else if (next_point == (current_point-1))
            {   ROS_WARN_STREAM(__LINE__);
                double distance=0.4;
                ROS_INFO_STREAM("distance "<<distance);
                if(pa.error <= distance)
                {
                    velocity.linear.x=0;
                    velocity.linear.y=0.05;
                    pa.vel_pub.publish(velocity);

                    //ROS_INFO_STREAM("current_x "<<pa.current_x);
                    pa.error =std::sqrt(std::pow((pa.current_x-pa.old_x),2)+std::pow((pa.current_y-pa.old_y),2));
                    //ROS_INFO_STREAM("error "<<pa.error);
                }
                else
                {
                    velocity.linear.x=0;
                    velocity.linear.y=0;
                    pa.vel_pub.publish(velocity);
                    pa.step_number++;
                    pa.error=0;
                    pa.old_x = pa.current_x;
                    pa.old_y = pa.current_y;
                    ROS_INFO_STREAM("error "<<pa.error);
                }

            }
            else if (next_point == (current_point+1))
            {   ROS_WARN_STREAM(__LINE__);
                double distance=0.4;
                ROS_INFO_STREAM("distance "<<distance);
                if(pa.error <= distance)
                {
                    velocity.linear.x=0;
                    velocity.linear.y=-0.05;
                    pa.vel_pub.publish(velocity);

                    //ROS_INFO_STREAM("current_x "<<pa.current_x);
                    pa.error =std::sqrt(std::pow((pa.current_x-pa.old_x),2)+std::pow((pa.current_y-pa.old_y),2));
                    //ROS_INFO_STREAM("error "<<pa.error);
                }
                else
                {
                    velocity.linear.x=0;
                    velocity.linear.y=0;
                    pa.vel_pub.publish(velocity);
                    pa.step_number++;
                    pa.error=0;
                    pa.old_x = pa.current_x;
                    pa.old_y = pa.current_y;
                    ROS_INFO_STREAM("error "<<pa.error);
                }

            }
            else if (next_point == (current_point+11))
            {   ROS_WARN_STREAM(__LINE__);
                double distance=std::sqrt(2)*0.4;
                ROS_INFO_STREAM("distance "<<distance);
                if(pa.error <= distance)
                {
                    velocity.linear.x=-0.06;
                    velocity.linear.y=0.05;
                    pa.vel_pub.publish(velocity);

                    //ROS_INFO_STREAM("current_x "<<pa.current_x);
                    pa.error =std::sqrt(std::pow((pa.current_x-pa.old_x),2)+std::pow((pa.current_y-pa.old_y),2));
                    //ROS_INFO_STREAM("error "<<pa.error);
                }
                else
                {
                    velocity.linear.x=0;
                    velocity.linear.y=0;
                    pa.vel_pub.publish(velocity);
                    pa.step_number++;
                    pa.error=0;
                    pa.old_x = pa.current_x;
                    pa.old_y = pa.current_y;
                    ROS_INFO_STREAM("error "<<pa.error);
                }

            }
            else if (next_point == (current_point+12))
            {   ROS_WARN_STREAM(__LINE__);
                double distance=0.4;
                ROS_INFO_STREAM("distance "<<distance);
                if(pa.error <= distance)
                {
                    velocity.linear.x=-0.06;
                    velocity.linear.y=0;
                    pa.vel_pub.publish(velocity);

                    //ROS_INFO_STREAM("current_x "<<pa.current_x);
                    pa.error =std::sqrt(std::pow((pa.current_x-pa.old_x),2)+std::pow((pa.current_y-pa.old_y),2));
                    //ROS_INFO_STREAM("error "<<pa.error);
                }
                else
                {
                    velocity.linear.x=0;
                    velocity.linear.y=0;
                    pa.vel_pub.publish(velocity);
                    pa.step_number++;
                    pa.error=0;
                    pa.old_x = pa.current_x;
                    pa.old_y = pa.current_y;
                    ROS_INFO_STREAM("error "<<pa.error);
                }

            }
            else if (next_point == (current_point+13))
            {   ROS_WARN_STREAM(__LINE__);
                double distance=std::sqrt(2)*0.4;
                ROS_INFO_STREAM("distance "<<distance);
                if(pa.error <= distance)
                {
                    velocity.linear.x=-0.06;
                    velocity.linear.y=-0.05;
                    pa.vel_pub.publish(velocity);

                    //ROS_INFO_STREAM("current_x "<<pa.current_x);
                    pa.error =std::sqrt(std::pow((pa.current_x-pa.old_x),2)+std::pow((pa.current_y-pa.old_y),2));
                    //ROS_INFO_STREAM("error "<<pa.error);
                }
                else
                {
                    velocity.linear.x=0;
                    velocity.linear.y=0;
                    pa.vel_pub.publish(velocity);
                    pa.step_number++;
                    pa.error=0;
                    pa.old_x = pa.current_x;
                    pa.old_y = pa.current_y;
                    ROS_INFO_STREAM("error "<<pa.error);
                }

            }
            else
            {

            }
            ROS_INFO_STREAM("step_number is "<<pa.step_number);



        }

        loop_rate.sleep();
    }

    return 0;



}
