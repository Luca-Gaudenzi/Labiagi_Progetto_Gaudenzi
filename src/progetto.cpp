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

#define CONSTANT_LINEAR 500
#define CONSTANT_ANGULAR 55

#define GOAL_REPULSIVE_CONSTANT_LINEAR 60//200
#define GOAL_REPULSIVE_CONSTANT_ANGULAR 90
#define GOAL_ATTRACTIVE_CONSTANT_LINEAR 5
#define MAX_DIST 1
//using Isometry2f=Isometry_<float, 2>;

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
void Goal_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    goal_arrived=true;
    goal_x=msg->pose.position.x;
    goal_y=msg->pose.position.y;
    target_x=0.2;
    ROS_INFO("DESTINAZIONE: (%f %f)\n", goal_x, goal_y);
}
void Prog_Cmd_vel_Callback(const progetto::Prog_Cmd_vel::ConstPtr& msg){
    ROS_INFO("entrato %f %f\n", msg->vx, msg->vy);
    cmd_vel_arrived=true;
    vel_x=msg->vx;
    vel_y=msg->vy;
    angular=msg->angular;
    //determiniamo la posizione target con il comando di velocità preso in input
    target_x=vel_x*0.2;
    target_y=vel_y*0.2;
}


void Laser_Goal_Callback(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
    if(goal_arrived==false) return; //se ho ricevuto un goal, allora eseguo questa procedura
    //ricaviamo le informazioni per il calcolo del vettore velocità
    //preparo le istanze degli oggetti che userò per ricavare queste informazioni
    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    tf::TransformListener listener_goal;
    tf::StampedTransform transform_obstacle;
    tf::StampedTransform transform_goal;

    //ricavo la nuvola di punti dal laser scanner
    projector.transformLaserScanToPointCloud("base_laser_link", *laser_msg, cloud, listener);
    
    //ricavo le trasformate che userò per ottenere i punti della nuvola rispetto al robot e la posizione del goal rispetto al robot
    try{
        //questa conversioen in questo caso è poco utile, in quanto laser e robot sono molto vicini
        listener.waitForTransform("base_footprint", "base_laser_link", ros::Time(0), ros::Duration(10.0));
        listener.lookupTransform("base_footprint", "base_laser_link", ros::Time(0), transform_obstacle);

        listener_goal.waitForTransform("base_footprint", "map", ros::Time(0), ros::Duration(10.0));
        listener_goal.lookupTransform("base_footprint", "map", ros::Time(0), transform_goal);
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    //ricavo le matrici che definiscono la trasformazione per ricavare i punti nel sistema di riferimento del robot
    Eigen::Isometry2f laser_transform = convertPose2D(transform_obstacle);
    Eigen::Isometry2f goal_transform = convertPose2D(transform_goal);
    //inizializzo le forze repulsive dei punti individuati dal laser scanner (ostacoli)
    float sum_x=0; 
    float sum_y=0; //poichè il robot considerato è non olotomico, questa componente influenzerà solamente la rotazione del robot
    Eigen::Vector2f p;
    //ricavo le forze repulsive dalla nuvola dei punti
    for(auto& point: cloud.points){
        p(0)=point.x;
        p(1)=point.y;
        //ROS_INFO("untrasformed %f %f\n", p(0), p(1));
        p=laser_transform * p; //ricavo il punto ostacolo nel sistema riferimento del robot
        float modulo= 1/sqrt(point.x*point.x+point.y*point.y); //il modulo della forza repulsiva dipende dalla distanza dall'ostacolo
        //ROS_INFO("trasformed %f %f\n", p(0), p(1));
        sum_x+=p(0) * modulo * modulo;
        sum_y+=p(1) * modulo * modulo;
        //la direzione della forza repulsiva è quella del vettore robot-punto ostacolo (il verso è opposto)
        
        //determiniamo la posizione target
    }
    p(0) = goal_x;
    p(1) = goal_y;
    p = goal_transform * p;
    float robot_goal_x=p(0);
    float robot_goal_y=p(1);

    float distanza_rob_target=sqrt(target_x * target_x + target_y * target_y);
    ROS_INFO("distanza rob_target %f\n", distanza_rob_target);
    //le forze che consideriamo hanno stessa direzione e verso opposto dei vettori robot-punto collisione
    //quindi prendiamo l'opposto delle forze calcolate
    sum_x=-sum_x;
    sum_y=-sum_y;
    
    geometry_msgs::Twist msg_send;
    //le velocità date in input sono un peso per le forze ricavate dai punti ostacoli, in questa maniera all' aumentare o al diminuire della velocità in input cambiano anche le forze dei punti
    //così dovremmo essere in grado di fermarci davanti ad un ostacolo a prescindere dalla velocità che abbiamo
    //in questo caso la velocità in input considerata è quella derivante dalla forza attrattiva del punto goal

    

    float distanza_robot_goal=sqrt(robot_goal_y * robot_goal_y + robot_goal_x * robot_goal_x);
    ROS_INFO("distanza robot-goal %f\n", distanza_robot_goal);
    if(distanza_robot_goal < MAX_DIST){
        ROS_INFO("ARRIVATO\n");
        goal_arrived=false;
        return;
    }
    //dalla posizione del goal rispetto al robot ricavo la velocità attrattiva verso il goal
    float vel_goal_x =robot_goal_x / GOAL_ATTRACTIVE_CONSTANT_LINEAR;
    float vel_goal_y =robot_goal_y / GOAL_ATTRACTIVE_CONSTANT_LINEAR;
    float goal_scalar_prod=(target_x * vel_goal_x) + (target_y * vel_goal_y);
    float goal_cos=goal_scalar_prod/(distanza_rob_target * sqrt(vel_goal_y * vel_goal_y + vel_goal_x * vel_goal_x));
    float goal_sin=sqrt(1- goal_cos * goal_cos);
    if(vel_goal_y<0) goal_sin=-goal_sin;
    float goal_angular_vel = distanza_rob_target * sqrt(vel_goal_y * vel_goal_y + vel_goal_x * vel_goal_x) * goal_sin;
    ROS_INFO("goal_cos %f\n", goal_cos);
    ROS_INFO("goal_sin %f\n", goal_sin);
    ROS_INFO("goal_angular_vel %f\n", goal_angular_vel);
    //la forza attrattiva avrà come direzione e verso (robot_goal_x, robot_goal_y)
    //la componente y influirà solamente sulla direzione del robot essend non olotomico
    //ROS_INFO("posizione goal untrasformed: %f %f\n", goal_x, goal_y);
    //ROS_INFO("posizione goal trasformed: %f %f\n", robot_goal_x, robot_goal_y);
    ROS_INFO("FORZA ATTRATTIVA: %f %f\n", vel_goal_x, vel_goal_y);
    ROS_INFO("DISTANZA DAL GOAL: %f %f\n", robot_goal_x, robot_goal_y);
    //agiamo sulla componente angolare, l'idea è di usare i momenti angolari calcolati sul punto target(punto al quale si arriverebbe senza influenze delle forze repulsive)
    
    

    //calcolo seno tra due vettori
    float scalar_prod=(target_x * sum_x) + (target_y * sum_y);
    
    float cos=scalar_prod/(distanza_rob_target * sqrt(sum_x*sum_x + sum_y*sum_y));
    
    float sin=sqrt(1-cos*cos);
    
    if(sum_y<0) sin=-sin;
    //calcolo velocità angolare
    msg_send.angular.z=distanza_rob_target * sqrt(sum_x * sum_x + sum_y * sum_y) * sin;
    
    msg_send.angular.z/=GOAL_REPULSIVE_CONSTANT_ANGULAR;   
    ROS_INFO("angular repulsive %f\n", msg_send.angular.z );
    ROS_INFO("angular attractive %f\n", goal_angular_vel );
    //if(msg_send.angular.z<10 && msg_send.angular.z>-10) msg_send.angular.z=0;
    //agiamo sulle componenti x e y della velocità del robot
    sum_x/= GOAL_REPULSIVE_CONSTANT_LINEAR; 
    sum_y*=vel_goal_y / GOAL_REPULSIVE_CONSTANT_LINEAR;
    ROS_INFO("FORZA REPULSIVA %f %f\n", sum_x, sum_y);
    
    //ora calcoliamo l'effetto attrattivo del punto di goal
    //calcoliamo le coordinate del punto di gol rispetto al frame del robot
    
    msg_send.linear.x=sum_x + vel_goal_x;

    //msg_send.linear.y=sum_y/CONSTANT+vel_y;

    msg_send.angular.z+=goal_angular_vel;
    //MOMENTANEO
    //msg_send.angular.z=0;
    //MOMENTANEO
    ROS_INFO("comando velocità: %f %f %f\n", msg_send.linear.x, msg_send.linear.y, msg_send.angular.z);

    if(msg_send.angular.z<0.1 && msg_send.angular.z>-0.1) msg_send.angular.z=0;
    vel_pub.publish(msg_send);
}

void Laser_Cmd_vel_Callback(const sensor_msgs::LaserScan::ConstPtr& laser_msg){
    if(cmd_vel_arrived==false) return;
    cmd_vel_arrived=false;
    
    sensor_msgs::PointCloud cloud;
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;

    projector.transformLaserScanToPointCloud("base_laser_link", *laser_msg, cloud, listener);
    
    tf::StampedTransform transform_obstacle;

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

    Eigen::Isometry2f laser_transform = convertPose2D(transform_obstacle);
    
    float sum_x=0;
    float sum_y=0;
    Eigen::Vector2f p;
    for(auto& point: cloud.points){
        
        //float distanza=1/modulo;
        p(0)=point.x;
        p(1)=point.y;
        //ROS_INFO("untrasformed %f %f\n", p(0), p(1));
        p=laser_transform * p;
        //float modulo= 1/sqrt(p(0) * p(0) + p(1) * p(2));
        float modulo= 1/sqrt(point.x*point.x+point.y*point.y);
        //ROS_INFO("trasformed %f %f\n", p(0), p(1));
        sum_x+=p(0) * modulo * modulo;
        sum_y+=p(1) * modulo * modulo;
        
        
        //determiniamo la posizione target
    }
    //le forze che consideriamo hanno stessa direzione e verso opposto dei vettori robot-punto collisione
    //quindi prendiamo l'opposto delle forze calcolate
    sum_x=-sum_x;
    sum_y=-sum_y;
    geometry_msgs::Twist msg_send;
    //le velocità date in input sono un peso per le forze ricavate dai punti ostacoli, in questa maniera all' aumentare o al diminuire della velocità in input cambiano anche le forze dei punti
    //così dovremmo essere in grado di fermarci davanti ad un ostacolo a prescindere dalla velocità che abbiamo
    //agiamo sulla componente angolare, l'idea è di usare i momenti angolari calcolati sul punto target
    
    float distanza_rob_target=sqrt(target_x*target_x + target_y*target_y);
    ROS_INFO("la distanza vale %f\n", distanza_rob_target);
    //calcolo seno tra due vettori
    float scalar_prod=(target_x * sum_x) + (target_y * sum_y);
    float cos=scalar_prod/(distanza_rob_target * sqrt(sum_x*sum_x + sum_y*sum_y));
    float sin=sqrt(1-cos*cos);

    if(sum_y<0) sin=-sin;
    //calcolo velocità angolare
    msg_send.angular.z=distanza_rob_target * sqrt(sum_x*sum_x + sum_y*sum_y) * sin;
    msg_send.angular.z/=CONSTANT_ANGULAR;   
    
    //if(msg_send.angular.z<10 && msg_send.angular.z>-10) msg_send.angular.z=0;
    //agiamo sulle componenti x e y della velocità del robot
    sum_x*=vel_x; 
    sum_y*=vel_y;
    ROS_INFO("forze repulsive: %f %f\n", sum_x, sum_y);
    
    //ora calcoliamo l'effetto attrattivo del punto di goal
    //calcoliamo le coordinate del punto di gol rispetto al frame del robot

    msg_send.linear.x=sum_x/CONSTANT_LINEAR+vel_x;

    //msg_send.linear.y=sum_y/CONSTANT+vel_y;

    msg_send.angular.z+=angular;
    ROS_INFO("%f %f %f\n", msg_send.linear.x, msg_send.linear.y, msg_send.angular.z);
    //if(msg_send.angular.z<0.1 && msg_send.angular.z>-0.1) msg_send.angular.z=0;
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

    ros::Subscriber laser_sub= n.subscribe(argv[2], 1000, Laser_Callback);
    ROS_INFO("subscriber su %s avviato\n", argv[2]);
    
    ROS_INFO("subscriber su /Prog_Cmd_vel avviato\n");
    ros::Subscriber vel_sub=n.subscribe("/Prog_Cmd_vel", 1000, Prog_Cmd_vel_Callback);
    ROS_INFO("inviare messaggi su topic /Prog_Cmd_vel per far muovere il robot\n");

    ros::Subscriber goal_sub=n.subscribe("/move_base_simple/goal", 1000, Goal_Callback);
    ROS_INFO("ricezione messaggi avviata\n");
    ros::spin();
    return 0;
}
