#include <ros/ros.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <iterator>


int grid_value[14][12]={0};
//bool grid[14][12]={false};
std::vector<int> unwalkable(0);

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
//    ROS_INFO("rows %d ", msg->info.height);
//    ROS_INFO("colums %d", msg->info.width);

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
    //ROS_INFO_STREAM(unwalkable.size());

}

//    ROS_INFO("(0,2) (0,3) (0,4) (0,5) (0,6) (0,7) (0,8) %d %d %d %d %d %d %d", grid[0][2],grid[0][3],grid[0][4], grid[0][5],grid[0][6],grid[0][7],grid[0][8] );
//    ROS_INFO("(1,1) (1,2) (1,5) (1,6) (1,9) (1,11) %d %d %d %d %d %d", grid[1][1],grid[1][2],grid[1][5], grid[1][6],grid[1][9],grid[1][11] );
//    ROS_INFO("(2,1) (2,3) (2,9) (2,10) (2,11)  %d %d %d %d %d", grid[2][1],grid[2][3],grid[2][9], grid[2][10],grid[2][11]);
//    ROS_INFO("(3,1) (3,2) (3,11) %d %d %d", grid[3][1],grid[3][2],grid[3][11]);
//    ROS_INFO("(4,1) (4,11) %d %d", grid[1][1],grid[4][11]);




std::vector<int> G(168,0);
std::vector<int> H(168,0);
std::vector<int> F(168,0);
std::vector<int> parent(168,0);

std::vector<int> close_list(0);
std::vector<int> open_list(0);

int index_start=12*10+4;
int index_end=12*4+5;



void get_H(int index)
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

std::vector<int> get_F(std::vector<int> v1, std::vector<int> v2)
{
    std::vector<int> v3(v1.size());
    for(int i=0;i<v1.size();i++)
    {
        v3.at(i)=v1.at(i)+v2.at(i);
    }
    return v3;
}

void get_initial_opG(int index_start)//this function calculates the initial open_list and parent and G
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

std::vector<int> get_opG(int index_now, std::vector<int> open_list, std::vector<int> close_list, std::vector<int> unwalkable)//this function calculates the open_list and parent after the first time step
{

    if ( std::find(std::begin(close_list), std::end(close_list), index_now-13) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now-13) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now-13) == std::end(open_list))
        {
            open_list.push_back(index_now-13);
            parent.at(index_now-13)=index_now;
        }
        else
        {
            if(G.at(index_now-13)>G.at(index_now)+14)
            {ROS_WARN_STREAM(__LINE__);
                G.at(index_now-13)=G.at(index_now)+14;
                parent.at(index_now-13)=index_now;
            }

        }
    }
    if ( std::find(std::begin(close_list), std::end(close_list), index_now-12) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now-12) == std::end(unwalkable))
    {
        if (std::find(std::begin(open_list), std::end(open_list), index_now-12) == std::end(open_list))
        {
            open_list.push_back(index_now-12);
            parent.at(index_now-12)=index_now;
        }
        else
        {
            if(G.at(index_now-12)>G.at(index_now)+10)
            {ROS_WARN_STREAM(__LINE__);
                G.at(index_now-12)=G.at(index_now)+10;
                parent.at(index_now-12)=index_now;
            }

        }

    }
    if ( std::find(std::begin(close_list), std::end(close_list), index_now-11) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now-11) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now-11) == std::end(open_list))
        {
            open_list.push_back(index_now-11);
            parent.at(index_now-11)=index_now;
        }
        else
        {
            if(G.at(index_now-11)>G.at(index_now)+14)
            {ROS_WARN_STREAM(__LINE__);
                G.at(index_now-11)=G.at(index_now)+14;
                parent.at(index_now-11)=index_now;
            }

        }
    }
    if ( std::find(std::begin(close_list), std::end(close_list), index_now-1) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now-1) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now-1) == std::end(open_list))
        {
            open_list.push_back(index_now-1);
            parent.at(index_now-1)=index_now;
        }
        else
        {
            if(G.at(index_now-1)>G.at(index_now)+10)
            {ROS_WARN_STREAM(__LINE__);
                G.at(index_now-1)=G.at(index_now)+10;
                parent.at(index_now-1)=index_now;
            }

        }
    }
    if ( std::find(std::begin(close_list), std::end(close_list), index_now+1) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now+1) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now+1) == std::end(open_list))
        {
            open_list.push_back(index_now+1);
            parent.at(index_now+1)=index_now;
        }
        else
        {
            if(G.at(index_now+1)>G.at(index_now)+10)
            {ROS_WARN_STREAM(__LINE__);
                G.at(index_now+1)=G.at(index_now)+10;
                parent.at(index_now+1)=index_now;
            }

        }
    }
    if (std::find(std::begin(close_list), std::end(close_list), index_now+11) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now+11) == std::end(unwalkable))
    {

        if(std::find(std::begin(open_list), std::end(open_list), index_now+11) == std::end(open_list))
        {
            open_list.push_back(index_now+11);
            parent.at(index_now+11)=index_now;
        }
        else
        {
            if(G.at(index_now+11)>G.at(index_now)+14)
            {ROS_WARN_STREAM(__LINE__);
                G.at(index_now+11)=G.at(index_now)+14;
                parent.at(index_now+11)=index_now;
            }

        }

    }
    if (std::find(std::begin(close_list), std::end(close_list), index_now+12) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now+12) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now+12) == std::end(open_list))
        {
            open_list.push_back(index_now+12);
            parent.at(index_now+12)=index_now;
        }
        else
        {
            if(G.at(index_now+12)>G.at(index_now)+10)
            {ROS_WARN_STREAM(__LINE__);
                G.at(index_now+12)=G.at(index_now)+10;
                parent.at(index_now+12)=index_now;
            }

        }
    }
    if (std::find(std::begin(close_list), std::end(close_list), index_now+13) == std::end(close_list) && std::find(std::begin(unwalkable), std::end(unwalkable), index_now+13) == std::end(unwalkable))
    {
        if(std::find(std::begin(open_list), std::end(open_list), index_now+13) == std::end(open_list))
        {
            open_list.push_back(index_now+13);
            parent.at(index_now+13)=index_now;
        }
        else
        {
            if(G.at(index_now+13)>G.at(index_now)+14)
            {ROS_WARN_STREAM(__LINE__);
                G.at(index_now+13)=G.at(index_now)+14;
                parent.at(index_now+13)=index_now;
            }

        }
    }
    return open_list;



}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"analysemap");
    ros::NodeHandle n;
    ros::Subscriber map_sub=n.subscribe("/map", 1000, mapCallback);


    //first step
    close_list.push_back(index_start);
    get_H(index_end);//H stays the same
    get_initial_opG(index_start);
    F=get_F(G,H);

//    for(int i=0;i<open_list.size();i++)
//         {    //ROS_WARN_STREAM(__LINE__);
//              ROS_INFO_STREAM(i<<":"<<open_list.at(i));
//         }


    int next_index=open_list.at(0);
    for(int i=0;i<open_list.size()-1;i++)
    {

        if(F.at(open_list.at(i+1))<F.at(next_index))
        {
            next_index=open_list.at(i+1);
        }
        else
        {

        }

    }
    ROS_INFO_STREAM("next index "<<next_index);

    close_list.push_back(next_index);

    //remove next_index from open_list
    std::vector<int>::iterator position = std::find(open_list.begin(), open_list.end(), next_index);
    if (position != open_list.end())
    {
        open_list.erase(position);
    }
    ROS_INFO_STREAM("size of open_list "<<open_list.size());

    open_list=get_opG(next_index,open_list,close_list,unwalkable);

//    ROS_INFO_STREAM("size of open_list after call function "<<open_list.size());

//    for(int i=0;i<open_list.size();i++)
//       {
//           ROS_INFO_STREAM(i<<":"<<open_list.at(i));
//       }


    ros::spin();
    return 0;



}
