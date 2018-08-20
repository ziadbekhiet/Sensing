/**
 * @file application.cpp
 *
 * @copyright Software License Agreement (BSD License)
 * Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick 
 * All Rights Reserved.
 * For a full description see the file named LICENSE.
 *
 * Authors: Andrew Dobson, Andrew Kimmel, Athanasios Krontiris, Zakary Littlefield, Kostas Bekris
 *
 * Email: pracsys@googlegroups.com
 */

#include "prx/utilities/applications/application.hpp"
#include "prx/utilities/parameters/parameter_reader.hpp"
#include "prx/utilities/definitions/string_manip.hpp"
#include "prx/utilities/communication/communication.hpp"

#include "prx/utilities/communication/tf_broadcaster.hpp"

// #include <boost/range/adaptor/map.hpp> //adaptors
#include <fstream>
#include "prx/utilities/definitions/sys_clock.hpp"
#include <pluginlib/class_list_macros.h>
#include "prx_core/send_plants_srv.h"
#include "prx_core/visualize_obstacles_srv.h"
#include "prx/utilities/definitions/random.hpp"
#include <boost/assign/list_of.hpp>
#include "prx/utilities/graph/undirected_node.hpp"
#include "prx_core/take_screenshot_srv.h"

#include <iostream>
#include <map>
#include <vector>
#include <iterator>
#include <algorithm>

#include <string>
#include <sstream>
#include <deque>
using namespace std;

PLUGINLIB_EXPORT_CLASS(prx::util::util_application_t, prx::util::util_application_t)

namespace prx
{

    namespace util
    {
        //This is needed for pracsys to be able to load this class when we start a node.
        pluginlib::ClassLoader<util_application_t> util_application_t::loader("prx_core", "prx::util::util_application_t");

        util_application_t::util_application_t()
        {
            tf_broadcaster = NULL;
            agent_state = START;
            start_i = 0;
            start_j = 0;
            searcher = new search_t();
            init_random(1);
        }

        util_application_t::~util_application_t()
        {
            delete tf_broadcaster;
            delete searcher;
        }

        void util_application_t::init(const parameter_reader_t * const reader)
        {

            //create the tf broadcaster, which tells the visualization node where all of the geometries are placed in the world
            tf_broadcaster = new tf_broadcaster_t;
            ros::ServiceClient plant_client = node_handle.serviceClient<prx_core::send_plants_srv > ("visualization/plants");

            prx_core::send_plants_srv plant_wrapper;
            plant_wrapper.request.source_node_name = "utilities";
            plant_wrapper.request.paths.push_back("simulator/disk");
            plant_wrapper.request.paths.push_back("simulator/goal");
            plant_client.waitForExistence(ros::Duration(5));
            if( !plant_client.call(plant_wrapper) )
            {
                PRX_FATAL_S("Service request to send plants failed.");
            }

            //set the initial location of the robot
            auto start_pose = pose_from_indices(start_i, start_j);
            _x = start_pose.first;
            _y = start_pose.second;

            current_goal_x = _x;
            current_goal_y = _y;

            _z = 0.5;

            _qx = _qy = _qz = 0;
            _qw = 1;

            //initialize the state memory for the ball (where the robot can go)
            std::vector<double*> state_memory = {&_x, &_y, &_z, &_qx, &_qy, &_qz, &_qw};

            state_space = new space_t("SE3", state_memory);
            //The ball can only move within these bounds
            state_space->get_bounds()[0]->set_bounds(-r-2, r+2);
            state_space->get_bounds()[1]->set_bounds(-c-2, c+2);
            state_space->get_bounds()[2]->set_bounds(-2, 4);


            ros::ServiceClient obs_client = node_handle.serviceClient<prx_core::visualize_obstacles_srv > ("visualization/visualize_obstacles");
            prx_core::visualize_obstacles_srv obs_wrapper;
            obs_wrapper.request.obstacles_path = "utilities";
            obs_client.waitForExistence(ros::Duration(5));
            if( !obs_client.call(obs_wrapper) )
            {
                PRX_FATAL_S("Service request to send obstacles failed.");
            }

        }

        std::pair<int, int> util_application_t::pose_from_indices(int i, int j)
        {
            return std::make_pair(j, -i);
        }

        std::pair<int, int> util_application_t::indices_from_pose(int x, int y)
        {
            return std::make_pair(-y, x);
        }

        void util_application_t::build_environment(std::string filename, std::string block_filename)
        {
            char* w = std::getenv("PRACSYS_PATH");
            std::string block_file(w);
            block_file += ("/prx_core/launches/"+block_filename);
            PRX_PRINT("The obstacle template is "<<block_file, PRX_TEXT_RED);

            environment_file = filename;
            PRX_INFO_S("File directory is: " << filename);
            std::ifstream fin;

            std::string script_file(w);
            script_file += "/prx_core/prx/utilities/applications/create_environment.py"; 
            std::vector< std::tuple<int,int,int> > locations;


            try
            {
                fin.open(filename);
                if(!fin.good())
                {
                    PRX_FATAL_S("Error in trying to read file "<<filename);
                }
                PRX_PRINT("Opened file..", PRX_TEXT_MAGENTA);

                fin >> r;
                fin >> c;
                PRX_PRINT("The read attributes: "<<r<<" "<<c<<" ", PRX_TEXT_MAGENTA);

                maze.resize(r);
                int num_obs = 1;
                int total_cells = r*c;
                int progress = 0;

                for(int i=0; i<r; ++i)
                {
                    maze[i].resize(c,-1);
                    for(int j=0; j<c; ++j, ++progress)
                    {
                        // PRX_PRINT("Reading index: "<<i<<", "<<j<<" to "<<maze[i][j], PRX_TEXT_MAGENTA);
                        fin >> maze[i][j];
                        // std::cout<<maze[i][j]<<" ";
                        if(maze[i][j] == 0)
                        {
                            auto pose = pose_from_indices(i,j);
                            std::vector<double> obs_pos = {(double)pose.first, (double)pose.second, 0.5};
                            std::string command = "rosparam load "+block_file+" /utilities/obstacles/block_"+std::to_string(num_obs);
                            auto ret = std::system(command.c_str());
                            
                            

                            ros::param::set("/utilities/obstacles/block_"+std::to_string(num_obs)+"/root_configuration/position", obs_pos);
                            // ros::param::set("/utilities/obstacles/block_"+std::to_string(num_obs)+"/collision_geometry/filename", std::to_string(uniform_int_random(0,9)) + "_" + std::to_string(uniform_int_random(0,9)) + ".png" ); 


                            std::string script_command = "python "+script_file+" "+std::to_string(num_obs)+" "+std::to_string(uniform_int_random(0,9)) + "_" + std::to_string(uniform_int_random(0,9))+ "_" + std::to_string(uniform_int_random(0,9)) + ".obj";
                            auto script_ret = std::system(script_command.c_str());


                            if (j!=0 && maze[i][j-1]==1)
                                locations.push_back(std::make_tuple(i,j-1,num_obs));

                            num_obs++;
                        }
                        PRX_STATUS("Constructing environment: "<<progress*100/(double)total_cells<<"%       ", PRX_TEXT_LIGHTGRAY);
                    }
                    // std::cout<<"\n";
                }

            }
            catch(...)
            {
                PRX_FATAL_S("Could not parse file.");
            }




            std::vector<int> positions = {0,1,2,3,4,5,6,7,8,9};
            std::random_shuffle ( positions.begin(), positions.end() );
            std::random_shuffle ( locations.begin(), locations.end() );
            if(locations.size()>10)
                locations.resize(10);

            
            for(auto p:positions){digit_to_position[p] = std::make_pair(0,0);}for(int i = 0; i<locations.size(); ++i){digit_to_position[positions[i]] = std::make_pair(std::get<0>(locations[i]),std::get<1>(locations[i]));//PRX_PRINT("The digit: "<<positions[i]<<" mapped to "<<std::get<0>(locations[i])<<", "<<std::get<1>(locations[i]), PRX_TEXT_BROWN);
            }int i;for(i = 0; i < locations.size()-1 && i < 9; ++i) {   auto location = locations[i]; auto next_location = locations[i+1];int this_index = positions[i];int next_index = positions[i+1];
            std::string script_command = "python "+script_file+" "+std::to_string(std::get<2>(location))+" "+std::to_string(next_index) + "_" + std::to_string(this_index)+ "_" + std::to_string(uniform_int_random(0,9)) + ".obj";
            auto script_ret = std::system(script_command.c_str()); } std::string script_command = "python "+script_file+" "+std::to_string(std::get<2>(locations[i]))+" "+std::to_string(positions[0]) + "_" + std::to_string(positions[i])+ "_" + std::to_string(uniform_int_random(0,9)) + ".obj"; auto script_ret = std::system(script_command.c_str());
            std::random_shuffle ( positions.begin(), positions.end() );
            std::random_shuffle ( locations.begin(), locations.end() );
            

            PRX_PRINT("Printing the read maze:", PRX_TEXT_MAGENTA);
            std::cout<<"\n-----------------------------\n";
            for(int i=0; i<r; ++i)
            {
                std::cout<<"\n";
                for(int j=0; j<c; ++j)
                {
                    std::cout<<maze[i][j]<<" ";
                    if(! (maze[i][j]==0 || maze[i][j]==1) )
                    {
                        PRX_FATAL_S("Malformed maze. Cells can either be 0 or 1.");
                    }
                }
            }
            std::cout<<"\n-----------------------------\n";
        }

        void util_application_t::update_visualization()
        {
            PRX_ASSERT(tf_broadcaster != NULL);
            
            //update visualization about where the robot is
            robot_configuration.set_position(_x,_y,_z);
            robot_configuration.set_orientation(_qx,_qy,_qz,_qw);
            goal_configuration.set_position(current_goal_x,current_goal_y,_z);
            goal_configuration.set_orientation(_qx,_qy,_qz,_qw);
            tf_broadcaster->queue_config(robot_configuration, "simulator/disk/ball");
            tf_broadcaster->queue_config(goal_configuration, "simulator/goal/ball");

            
            //send every configuration we have added to the list
            tf_broadcaster->broadcast_configs();
        }

        void util_application_t::info_broadcasting(const ros::TimerEvent& event)
        {
            update_visualization();
            consumed_waypoint = true;
        }

        pluginlib::ClassLoader<util_application_t>& util_application_t::get_loader()
        {
            return loader;
        }


        void util_application_t::execute()
        {
            while(ros::ok())
            {
                if(agent_state == START)
                {
                    PRX_PRINT("Current state is START", PRX_TEXT_BROWN);
                    sleep(5);
                    agent_state = SENSE;
                }
                else if(agent_state == SENSE)
                {
                    ros::ServiceClient screenshot_client = node_handle.serviceClient<prx_core::take_screenshot_srv > ("visualization/take_screenshot");
                    prx_core::take_screenshot_srv screenshot_wrapper;
                    screenshot_wrapper.request.screen_num = 1;
                    screenshot_wrapper.request.number_of_screenshots = 1;
                    screenshot_client.waitForExistence(ros::Duration(5));
                    if( !screenshot_client.call(screenshot_wrapper) )
                    {
                        PRX_FATAL_S("Service request to send obstacles failed.");
                    }
                    char* w = std::getenv("PRACSYS_PATH");
                    std::string sensing_image(w);
                    sensing_image += ("/prx_output/images/_0.jpg");

                    PRX_PRINT("Current state is SENSE", PRX_TEXT_BROWN);
                    auto goal = sense(sensing_image);
                    current_goal_i = goal.first;
                    current_goal_j = goal.second;
                    agent_state = PLAN;
                    PRX_PRINT("Sensed: ["<<current_goal_i<<", "<<current_goal_j<<"]", PRX_TEXT_GREEN);
                    auto xy_goal = pose_from_indices(current_goal_i, current_goal_j);
                    current_goal_x = xy_goal.first;
                    current_goal_y = xy_goal.second;
                }
                else if(agent_state == PLAN)
                {
                    PRX_PRINT("Current state is PLAN", PRX_TEXT_BROWN);
                    auto plan_initial = indices_from_pose(_x,_y);
                    PRX_PRINT("Planning from ["<<plan_initial.first<<","<<plan_initial.second<<"] -> ["<<current_goal_i<<","<<current_goal_j<<"]", PRX_TEXT_CYAN);
                    current_path = plan(plan_initial.first, plan_initial.second, current_goal_i, current_goal_j);
                    PRX_PRINT("Path: ", PRX_TEXT_LIGHTGRAY);
                    for(auto p: current_path)
                        std::cout<<"["<<p.first<<","<<p.second<<"]->";
                    std::cout<<"[end]\n";
                    consumed_waypoint = true;
                    path_counter = 0;
                    agent_state = MOVE;
                }
                else if(agent_state == MOVE)
                {
                    if(consumed_waypoint)
                    {
                        PRX_STATUS("Current state is MOVE: ["<<indices_from_pose(_x,_y).first<<", "<<indices_from_pose(_x,_y).second<<"]                ", PRX_TEXT_BROWN);
                        if(path_counter < current_path.size())
                        {
                            move();
                            agent_state = MOVE;
                        }
                        else
                        {
                            agent_state = START;
                        }
                    }
                    else
                    {
                        agent_state = MOVE;
                    }
                }
                ros::spinOnce();
            }
        }


        void util_application_t::move()
        {
            auto current_coordinate = pose_from_indices(current_path[path_counter].first, current_path[path_counter].second);
            _x = current_coordinate.first;
            _y = current_coordinate.second;
            path_counter++;
            consumed_waypoint = false;
        }







        std::pair<int, int> util_application_t::sense(std::string sensing_image)
        {
            // Currently reports a random collision free state as a goal
            while(true)
            {
                int i = uniform_int_random(0,r-1);
                int j = uniform_int_random(0,c-1);
                if(maze[i][j] == 1)
                {
                    return std::make_pair(i,j);
                }
            }
        }

        //namespace std{
	int manhattan_distance(int ci, int cj, int di, int dj){
    
    		int x = abs(di - ci);
    		int y = abs(dj - cj);
    
    		return x+y;
	};
       vector< std::pair<int, int> > util_application_t::plan(int initial_i, int initial_j, int goal_i, int goal_j )
        {    
	int startx = initial_i;
	int starty = initial_j;
	int finishx = goal_i;
	int finishy = goal_j;

    // file reader
    int cnt = 0;
    int EX;
    int EY;
    int X[10][10];
    string line;
    string filelocation = "/home/ziad/prx_core_ws/src/prx_core/launches/test_maze.txt";
    ifstream myfile(filelocation);
    if (myfile.is_open()){
        while(getline(myfile,line)){
            cnt++;
            if(cnt == 1){
                stringstream convert(line);
                convert >> EX;
// prints read x                cout << X << endl;
            }
            else if(cnt == 2){
                stringstream convert2(line);
                convert2 >> EY;
// prints read y                cout << Y << endl;
            }
        }
        myfile.close();
    }
    X[EX][EY];
    cnt = 0;
    ifstream myfile2(filelocation);
    if (myfile2.is_open()){
        while(getline(myfile2,line)){
            cnt++;
            if(cnt > 2){
                stringstream c (line);
                int ycount = 0;
                while(1){
                    int n;
                    c >> n;
                    if(!c){
                        break;
                    }
                    X[cnt-3][ycount] = n;
                    ycount++;
                }
            }
        }
        myfile2.close();
    }
    //prints file reader input
 //   for(int i = 0; i < X; i++){
 //       for(int j = 0; j < Y; j++){
 //           cout << X[i][j] << " ";
 //       }
 //       cout << endl;
 //   }
    //end print file reader input
// end file reader
    vector<pair<int,int>> explored;
    explored.push_back(make_pair(startx,starty));
    
    
    map<pair<int,int>,int> val;
    
    for(int i = 0; i < 10; i++){
        for(int j =0; j < 10; j++){
            if(X[i][j] == 1){
                val[make_pair(i,j)] = 99;}
            if(X[i][j] == 0){
                val[make_pair(i,j)] = 0;
            }
        }
    }
    // creates a new map to keep track of parent nodes
    map<pair<int,int>,pair<int,int>> parent;
    
    parent[make_pair(startx,starty)] = make_pair(startx,starty);
    
    
    int current_nodex = startx;
    int current_nodey = starty;
    int f_g = 0;
    int pval = 0; // cost to reach the current node : this is my cost function
    val[make_pair(startx,starty)] = -1;
    vector<pair<int,int>> :: const_iterator iter;
    while(current_nodex != finishx || current_nodey != finishy){
        pair<int,int> pos;
        // unexplored node updater
        for(iter = explored.begin(); iter < explored.end(); iter++){
            int ex = iter -> first;
            int ey = iter -> second;
            int upy = ey + 1;
            if(upy>=9){
                upy = 9;
            };
            int upx = ex;
            
            int rx = ex + 1;
            int ry = ey;
            if(ry >= 9){
                ry = 9;
            }
            
            int dx = ex;
            int dy = ey - 1;
            if(dy <= 0){
                dy = 0;
            }
            
            int lx = ex-1;
            int ly= ey;
            if(lx <= 0){
                lx = 0;
            }
            
            if(val[make_pair(upx,upy)] > manhattan_distance(upx,upy,finishx,finishy) + -1*val[make_pair(ex,ey)] && val[make_pair(upx,upy)] !=0){
                val[make_pair(upx,upy)] = manhattan_distance(upx,upy,finishx,finishy) + -1*val[make_pair(current_nodex,current_nodey)];
                parent[make_pair(upx,upy)] = make_pair(ex,ey);
            }
            
            if(val[make_pair(dx,dy)] > manhattan_distance(dx,dy,finishx,finishy)+ -1*val[make_pair(ex,ey)] && val[make_pair(dx,dy)] !=0){
                val[make_pair(dx,dy)] = manhattan_distance(dx,dy,finishx,finishy) + -1*val[make_pair(ex,ey)];
                parent[make_pair(dx,dy)] = make_pair(ex,ey);
            }
            if(val[make_pair(rx,ry)] > manhattan_distance(rx,ry,finishx,finishy) + -1*val[make_pair(ex,ey)]&& val[make_pair(rx,ry)] !=0){
                val[make_pair(rx,ry)] = manhattan_distance(rx,ry,finishx,finishy) + -1*val[make_pair(ex,ey)];
                parent[make_pair(rx,ry)] = make_pair(ex,ey);
            }
            if(val[make_pair(lx,ly)] > manhattan_distance(lx,ly,finishx,finishy) + -1*val[make_pair(ex,ey)]&& val[make_pair(lx,ly)] !=0){
                val[make_pair(lx,ly)] = manhattan_distance(lx,ly,finishx,finishy) + -1*val[make_pair(ex,ey)];
                parent[make_pair(lx,ly)] = make_pair(ex,ey);
            }
        }
        
        
        int minv = 99;
        pair<int,int> minp = make_pair(current_nodex,current_nodey);
        for(int i = 0; i < 10; i++){
            for(int j =0; j < 10; j++){
                if (val[make_pair(i,j)] < minv && val[make_pair(i,j)] > 0){
                    minv = val[make_pair(i,j)];
                    minp = make_pair(i,j);
                }
            }
            //     cout << endl;
        }
        // if goal node can not be reached
        if(minv == 99 && minp == make_pair(current_nodex,current_nodey)){
            vector<pair<int,int>> empty;
            //outputs the final explored nodes number
            //unnecessary
            //int opened_nodes = 0;
            //  int explored_nodes = 0;
            //    for(int i = 0; i < 10; i++){
            //          for(int j =0; j < 10; j++){
            //                if(val[make_pair(i,j)] >= -1 && val[make_pair(i,j)]< 99){
            //          opened_nodes++;
            //        }
            //          if(val[make_pair(i,j)] < 0){
            //                explored_nodes++;
            //              }
            //          }
            //      }
            //      cout << endl << endl;
            //      for(int i = 0; i < 10; i++){
            //          for(int j =0; j < 10; j++){
            //              cout << val[make_pair(i,j)] << " ";
            //          }
            //          cout << endl;
            //      }
            //      cout << "on the map, -1 gives explored nodes and -2 represents an obstacle\n";
            //       cout << "opened nodes: " << opened_nodes << endl;
            //       cout << "explored nodes: " << explored_nodes << endl;
            // unnecessary
            return empty;
            break;
        }
        // end if goal node can not be reached
        
        for(int i = 0; i < 10; i++){
           for(int j =0; j < 10; j++){
 //                       cout << val[make_pair(i,j)] << " ";
            }
 //            cout << endl;
        }
 //       cout << endl << endl;
        explored.push_back(minp);
        //set the value of min p to the minimum value of neighbors minus 1
        val[minp] = val[parent[minp]] - 1;
        
        current_nodex = minp.first;
        current_nodey = minp.second;
    
        //    cout << "current_nodex: " << current_nodex << endl;
        //    cout << "current_nodey: " << current_nodey << endl;
        
    }
    
    int fringe_nodes = 0;
    int expanded_nodes = 0;
    //outputs the number of explored nodes
    for(int i = 0; i < 10; i++){
        for(int j =0; j < 10; j++){
            if(val[make_pair(i,j)] > 0 && val[make_pair(i,j)]< 99){
                fringe_nodes++;
            }
            if(val[make_pair(i,j)] < 0){
                expanded_nodes++;
            }
//richie            cout << val[make_pair(i,j)] << " ";
        }
//richie        cout << endl;

    }
    // outputs the final val matrix
 //   cout << endl << endl;
    for(int i = 0; i < 10; i++){
        for(int j =0; j < 10; j++){
            if(val[make_pair(i,j)] == 0){
                val[make_pair(i,j)] = -99;
            }
 //                     cout << val[make_pair(i,j)] << " ";
        }
 //             cout << endl;
    }
    //    cout << "on the map, -x gives explored nodes, 0 represents an obstacle and 99 represents an unexplored node\n";
//    cout << "fringe nodes: " << fringe_nodes << endl;
//    cout << "expanded nodes: " << expanded_nodes << endl;
    
    //redo search while moving along the most negative values to find the searh path.
    vector<pair<int,int>> final_path;
    current_nodex = finishx;
    current_nodey = finishy;
    final_path.push_back(make_pair(current_nodex,current_nodey));
    while(current_nodex != startx || current_nodey != starty){
        int upy = current_nodey + 1;
        if(upy>=9){
            upy = 9;
        }
        int upx = current_nodex;
        
        int rx = current_nodex + 1;
        int ry = current_nodey;
        if(ry >= 9){
            ry = 9;
        }
        
        int dx = current_nodex;
        int dy = current_nodey - 1;
        if(dy <= 0){
            dy = 0;
        }
        
        int lx = current_nodex-1;
        int ly= current_nodey;
        if(lx <= 0){
            lx = 0;
        }
        int valup = val[make_pair(upx,upy)];
        if (valup >= 0){
            valup = -99;
        }
        int vald = val[make_pair(dx,dy)];
        if (vald >= 0){
            vald = -99;
        }
        
        int vall = val[make_pair(lx,ly)];
        if (vall >= 0){
            vall = -99;
        }
        
        
        int valr = val[make_pair(rx,ry)];
        if (valr >= 0){
            valr = -99;
        }
        
        
        
        if (valup >= vald && valup >= vall && valup >= valr){
            current_nodex = upx;
            current_nodey = upy;
        }
        else if (vald >= valup && vald >= vall & vald >= valr){
            current_nodex = dx;
            current_nodey = dy;
        }
        else if (valr >= valup && valr >= vald && valr >= vall){
            current_nodex = rx;
            current_nodey = ry;}
        else if (vall >= valup && vall >= vald && vall >= valr){
            current_nodex = lx;
            current_nodey = ly;
        }
        final_path.push_back(make_pair(current_nodex,current_nodey));
//               cout << "(" << current_nodex << "," << current_nodey << ")";
    }
    // you were here before start
 //   cout << endl << endl;
 //   for(int i = 0; i < 10; i++){
 //       for(int j =0; j < 10; j++){
 //           cout << val[make_pair(i,j)] << " ";
 //       }
 //       cout << endl;
//}
    // you were here before end
    
    reverse(final_path.begin(), final_path.end());
    //    cout << endl<< endl;
    // vector<pair<int,int>>::const_iterator itf;
    int itf;
    int countt = 0;
    for(itf = 0; itf < final_path.size(); itf++){
//richie       cout << "(" << final_path[itf].first << "," << final_path[itf].second << ")" << endl;
        countt++;
    }
    
 //   cout << "Total steps: " << countt << endl;

    
    return final_path;


		
       }

    }
}
