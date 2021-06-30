#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "tf/transform_listener.h"
#include "progetto/Prog_Cmd_vel.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include "geometry_utils_fd.h"
#include <cmath>

#define CONSTANT_LINEAR 500
#define CONSTANT_ANGULAR 30//55

#define GOAL_REPULSIVE_CONSTANT_LINEAR 500//200
#define GOAL_REPULSIVE_CONSTANT_ANGULAR 55 // 55
#define GOAL_ATTRACTIVE_CONSTANT_LINEAR 1 //5
#define GOAL_ATTRACTIVE_CONSTANT_ANGULAR 1
#define MAX_DIST 1

ros::Publisher vel_pub;
float goal_x=0;
float goal_y=0;
float vel_x=0;
float vel_y=0;
float angular=0;
float target_x=0;
float target_y=0;
bool goal_arrived=false;
bool cmd_vel_arrived=false;

using namespace Eigen;

void Prog_Cmd_vel_Callback(const geometry_msgs::Twist::ConstPtr& msg){
    ROS_INFO("ricevuto comando movimento %f %f %f\n", msg->linear.x, msg->linear.y, msg->angular.z); 
    cmd_vel_arrived=true; //questa variabile settata a true fa in modo che all'arrivo di un messaggio m di velocità Prog_Cmd_vel, venga inviato un comando di velocità
                            //basato sul messaggio m, ma con alcune modifiche per evitare urti
    vel_x=msg->linear.x;
    vel_y=msg->linear.y; //in realtà, avendo considerato robot non olotomici, la componente y di movimento non è considerata
    angular=msg->angular.z;
    //determiniamo la posizione target nella quale calcoleremo i momenti rotazionali agenti sul robot
    target_x=vel_x*0.2; //
    //target_y=vel_y*0.2;
}



void Laser_Cmd_vel_Callback(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
    if(cmd_vel_arrived==false) return; //se non è arrivato nessun comando di velocità, questa procedura non deve essere eseguita
    cmd_vel_arrived=false; //visto che stiamo processando questo comando di velocità, settiamo la variabile a false per evitare di processarlo due volte
    
    //inizializzazione oggetti usati per la procedura
    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;

    //ottenimento punti ostacolo dal laser scanner
    projector.transformLaserScanToPointCloud("base_laser_link", *laser_msg, cloud, listener);
    
    tf::StampedTransform transform_obstacle;
    //otteniamo la trasformata per ottenere i punti ostacolo rispetto al robot
    try{
        //questa conversioen in questo caso è poco utile, in quanto laser e robot sono molto vicini
        listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), transform_obstacle);

    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    //otteniamo la trasformata sottoforma di matrice
    Eigen::Isometry2f laser_transform = convertPose2D(transform_obstacle);
    
    float sum_x=0;
    float sum_y=0;
    Eigen::Vector2f p;
    for(auto& point: cloud.points){
        
        p(0)=point.x;
        p(1)=point.y;
        //ROS_INFO("untrasformed %f %f\n", p(0), p(1));
        p=laser_transform * p;
        float modulo= 1/sqrt(point.x*point.x+point.y*point.y);
        //ROS_INFO("trasformed %f %f\n", p(0), p(1));
        sum_x+=p(0) * modulo * modulo;
        sum_y+=p(1) * modulo * modulo;
        
    }
    //le forze che consideriamo hanno stessa direzione e verso opposto dei vettori robot->punto-collisione
    //quindi prendiamo l'opposto delle forze calcolate
    sum_x=-sum_x;
    sum_y=-sum_y;
    geometry_msgs::Twist msg_send;
    //le velocità date in input sono un peso per le forze ricavate dai punti ostacoli, in questa maniera all' aumentare o al diminuire della velocità in input cambiano anche le forze dei punti
    //così dovremmo essere in grado di fermarci davanti ad un ostacolo a prescindere dalla velocità che abbiamo
    //agiamo sulla componente angolare, l'idea è di usare i momenti angolari calcolati sul punto target
    
    float distanza_rob_target=sqrt(target_x*target_x + target_y*target_y);
    //ROS_INFO("la distanza vale %f\n", distanza_rob_target);
    //calcolo seno tra due vettori
    /*float scalar_prod=(target_x * sum_x) + (target_y * sum_y);
    float cos=scalar_prod/(distanza_rob_target * sqrt(sum_x*sum_x + sum_y*sum_y));
    float sin=sqrt(1-cos*cos);

    if(sum_y<0) sin=-sin;*/
    //calcolo velocità angolare
    msg_send.angular.z=distanza_rob_target * sum_y / CONSTANT_ANGULAR;  
    ROS_INFO("forze angolari %f %f\n", msg_send.angular.z, angular);
    //agiamo sulle componenti x e y della velocità del robot
    sum_x*=abs (vel_x) / CONSTANT_LINEAR; 
    sum_y*=abs (vel_y) / CONSTANT_LINEAR;
    //ROS_INFO("forze repulsive: %f %f\n", sum_x, sum_y);
    

    msg_send.linear.x=sum_x + vel_x;
    
    //msg_send.linear.y=sum_y + vel_y;  opzione da usare se robot olotomico

    msg_send.angular.z+=angular;
    ROS_INFO("vettore velocita %f %f %f\n", msg_send.linear.x, msg_send.linear.y, msg_send.angular.z);
    //if(msg_send.angular.z<0.1 && msg_send.angular.z>-0.1) msg_send.angular.z=0;   //se vogliamo evitare piccole rotazioni durante il moto
    vel_pub.publish(msg_send);
}



//arrivato un messaggio del laserscanner, sulla base delle variabili booleane goal_arrived e cmd_vel_arrived, decido quale funzione eseguire
void Laser_Callback(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
    
    if(cmd_vel_arrived) Laser_Cmd_vel_Callback(laser_msg);
    if(goal_arrived) Laser_Goal_Callback(laser_msg);
    return;
}

int main(int argc, char **argv){
    //prendiamo gli input del programma
    if(argc<3){
        ROS_INFO("Dare in input: 1- topic comando di velocità da dare al robot\n2-laser scanner del robot\n");
        return -1;
    }
    
    ros::init(argc, argv, "progetto");
    
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    
    vel_pub=n.advertise<geometry_msgs::Twist>(argv[1], 1000);
    ROS_INFO("publisher su %s avviato\n", argv[1]);

    ros::Subscriber laser_sub= n.subscribe(argv[2], 5, Laser_Callback);
    ROS_INFO("subscriber su %s avviato\n", argv[2]);
    
    ROS_INFO("subscriber su /Prog_Cmd_vel avviato\n");
    ros::Subscriber vel_sub=n.subscribe("/Prog_Cmd_vel", 1, Prog_Cmd_vel_Callback);
    /*ROS_INFO("subscriber su /Prog_Cmd_vel_Converter avviato\n");
    ros::Subscriber converter_sub=n.subscribe("/Prog_Cmd_vel_Converter", 5, Prog_Cmd_vel_Converter_Callback);*/
    ROS_INFO("inviare messaggi su topic /Prog_Cmd_vel per far muovere il robot\n");

    //ros::Subscriber goal_sub=n.subscribe("/move_base_simple/goal", 5, Goal_Callback);
    ROS_INFO("ricezione messaggi avviata\n");
    ros::spin();
    return 0;
}
