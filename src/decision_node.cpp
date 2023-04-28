#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "std_msgs/Bool.h"

#define waiting_for_a_person 0
#define observing_the_person 1
#define rotating_to_the_person 2
#define moving_to_the_person 3
#define interacting_with_the_person 4
#define rotating_to_the_base 5
#define moving_to_the_base 6
#define resetting_orientation 7

#define frequency_expected 25
#define max_base_distance 6

#define rotation_epsilon M_PI/18
#define approach_epsilon 0.3
#define translation_epsilon 0.1
#define interaction_epsilon 1.5

#define person_moving_epsilon 0.025

class decision_node
{
private:
    ros::NodeHandle n;

    // communication with datmo_node
    ros::Subscriber sub_person_position;
    bool new_person_position, person_tracked, person_moving;
    geometry_msgs::Point person_position, last_person;

    // communication with robot_moving_node
    ros::Subscriber sub_robot_moving;
    bool robot_moving;

    // communication with rotation_action
    ros::Publisher pub_rotation_to_do;
    float rotation_to_person;

    // communication with action_node
    ros::Publisher pub_goal_to_reach;
    float translation_to_person;

    // communication with localization
    ros::Subscriber sub_localization;
    bool new_localization;
    bool init_localization;
    geometry_msgs::Point current_position;
    float current_orientation;
    float translation_to_base;
    float rotation_to_base;
    geometry_msgs::Point local_base_position;

    int current_state, previous_state;
    int frequency;
    geometry_msgs::Point base_position;
    float base_orientation;
    geometry_msgs::Point origin_position;
    bool state_has_changed;

public:
    decision_node()
    {

        // communication with datmo_node
        sub_person_position = n.subscribe("person_position", 1, &decision_node::person_positionCallback, this);

        // communication with rotation_node
        pub_rotation_to_do = n.advertise<geometry_msgs::Point>("goal_to_rotate", 0);

        // communication with action_node
        pub_goal_to_reach = n.advertise<geometry_msgs::Point>("goal_to_reach", 1); // Preparing a topic to publish the position of the person

        // communication with robot_moving_node
        sub_robot_moving = n.subscribe("robot_moving", 1, &decision_node::robot_movingCallback, this);

        // communication with localization node
        sub_localization = n.subscribe("localization", 1, &decision_node::localizationCallback, this);

        current_state = waiting_for_a_person;
        previous_state = -1;

        state_has_changed = false;

        // TO DEFINE according to the position of the base/initial position in the map
        base_position.x = 0;
        base_position.y = 0;
        base_orientation = 0;

        origin_position.x = 0;
        origin_position.y = 0;

        person_tracked = false;
        person_moving = false;

        // INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
        ros::Rate r(10); // this node will work at 10hz
        while (ros::ok())
        {
            ros::spinOnce(); // each callback is called once
            update();
            r.sleep(); // we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
        }
    }

    // UPDATE: main processing of laser data
    /*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
    void update()
    {

        //if (init_localization)
        //{

            update_variables();

            switch (current_state)
            {
            case waiting_for_a_person:
                process_waiting_for_a_person();
                break;

            case observing_the_person:
                process_observing_the_person();
                break;

            case rotating_to_the_person:
                process_rotating_to_the_person();
                break;

            case moving_to_the_person:
                process_moving_to_the_person();
                break;

            case interacting_with_the_person:
                process_interacting_with_the_person();
                break;

            case rotating_to_the_base:
                process_rotating_to_the_base();
                break;

            case moving_to_the_base:
                process_moving_to_the_base();
                break;

            case resetting_orientation:
                process_resetting_orientation();
                break;
            }

        if (translation_to_base > max_base_distance)
            current_state = rotating_to_the_base;

        new_localization = false;

        state_has_changed = current_state != previous_state;
        previous_state = current_state;
    //}
    //else ROS_WARN("Initialize localization");

} // update

void
update_variables()
{
    if (new_person_position)
    {
        translation_to_person = distancePoints(origin_position, person_position);

        if (translation_to_person > 0)
        {
            rotation_to_person = acos(person_position.x / translation_to_person);
            if (person_position.y < 0)
                rotation_to_person *= -1;
        }
        else
            rotation_to_person = 0;

        person_tracked = person_position.x != 0 || person_position.y != 0;
        person_moving = distancePoints(person_position, last_person) >= person_moving_epsilon;

        last_person.x = person_position.x;
        last_person.y = person_position.y;
    }

    if (new_localization)
    {
        // when we receive a new position(x, y, o) of robair in the map, we update:
        // local_base_position: the position of the base in the cartesian local frame of robot
        local_base_position.x = current_position.x - base_position.x;
        local_base_position.x = current_position.x - base_position.x;
        // translation_to_base: the translation that robair has to do to reach its base
        translation_to_base = distancePoints(current_position, base_position);
        // rotation_to_base: the rotation that robair has to do to reach its base
        rotation_to_base = current_orientation - base_orientation;
    }
}

void process_waiting_for_a_person()
{

    if (state_has_changed)
        ROS_WARN("current_state: waiting_for_a_person");

    // Processing of the state
    // as soon as we detect a moving person, we switch to the state "observing_the_person"
    if (new_person_position)
        current_state = observing_the_person;
}

void process_observing_the_person()
{

    if (state_has_changed)
    {
        ROS_WARN("current_state: observing_the_person");
        ROS_WARN("person_position: (%f, %f)", person_position.x, person_position.y);
        frequency = 0;
    }

    // Processing of the state
    // Robair only observes and tracks the moving person
    // if the moving person does not move during a while (use frequency), we switch to the state "rotating_to_the_person"
    if (new_person_position && person_moving)
    {
        frequency = 0;
    }
    else
        frequency++;

    ROS_WARN("observing_the_person: (%f, %f), %d", person_position.x, person_position.y, frequency);
    if (frequency >= frequency_expected)
        current_state = rotating_to_the_person;
}

void process_rotating_to_the_person()
{

    if (state_has_changed)
    {
        ROS_WARN("current_state: rotating_to_the_person");
        ROS_WARN("person_position: (%f, %f)", person_position.x, person_position.y);
        frequency = 0;
    }

    // Processing of the state
    // Robair rotates to be face to the moving person
    // if robair is face to the moving person and the moving person does not move during a while (use frequency), we switch to the state "moving_to_the_person"
    if (person_moving)
    {
        ROS_WARN("person_position: (%f, %f)", person_position.x, person_position.y);
        frequency = 0;
    }
    else
        frequency++;

    ROS_WARN("rotating_to_the_person: (%f, %f), %f, %d", person_position.x, person_position.y, rotation_to_person, frequency);
    pub_rotation_to_do.publish(person_position);
    if (!person_tracked)
        current_state = resetting_orientation;
    else if (frequency >= frequency_expected && abs(rotation_to_person) < rotation_epsilon)
        current_state = moving_to_the_person;
}

void process_moving_to_the_person()
{

    if (state_has_changed)
    {
        ROS_WARN("current_state: moving_to_the_person");
        ROS_WARN("person_position: (%f, %f)", person_position.x, person_position.y);
        frequency = 0;
    }

    // Processing of the state
    // Robair moves to be close to the moving person
    // if robair is close to the moving person and the moving person does not move during a while (use frequency), we switch to the state "interacting_with_the_person"
    if (person_moving)
    {
        ROS_WARN("person_position: (%f, %f)", person_position.x, person_position.y);
        frequency = 0;
    }
    else
        frequency++;

    ROS_WARN("moving_to_the_person: (%f, %f), %d", person_position.x, person_position.y, frequency);
    pub_goal_to_reach.publish(person_position);
    if (translation_to_base > max_base_distance || !person_tracked)
        current_state = rotating_to_the_base;
    else if (frequency >= frequency_expected && translation_to_person < approach_epsilon)
        current_state = interacting_with_the_person;
}

void process_interacting_with_the_person()
{

    if (state_has_changed)
    {
        ROS_WARN("current_state: interacting_with_the_person");
        ROS_WARN("person_position: (%f, %f)", person_position.x, person_position.y);
    }

    // Processing of the state
    // Robair does not move and interacts with the moving person until the moving person goes away from robair
    // if the person goes away from robair, after a while (use frequency), we switch to the state "rotating_to_the_base"
    if (new_person_position)
    {
        ROS_WARN("person_position: (%f, %f)", person_position.x, person_position.y);
    }

    if (translation_to_person > interaction_epsilon || !person_tracked)
        current_state = rotating_to_the_base;
}

void process_rotating_to_the_base()
{

    if (state_has_changed)
    {
        ROS_WARN("current_state: rotating_to_the_base");
        ROS_WARN("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
    }

    // Processing of the state
    // Robair rotates to be face to its base
    // if robair is face to its base and does not move, after a while (use frequency), we switch to the state "moving_to_the_base"
    if (new_localization)
    {
        ROS_WARN("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
    }

    std_msgs::Float32 rotation_to_base_msg;
    rotation_to_base_msg.data = rotation_to_person;
    pub_rotation_to_do.publish(rotation_to_base_msg);
    if (rotation_to_base < rotation_epsilon)
        current_state = moving_to_the_base;
}

void process_moving_to_the_base()
{

    if (state_has_changed)
    {
        ROS_WARN("current_state: moving_to_the_base");
        ROS_WARN("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
    }

    // Processing of the state
    // Robair moves to its base
    // if robair is close to its base and does not move, after a while (use frequency), we switch to the state "resetting_orientation"
    if (new_localization)
    {
        ROS_WARN("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
    }

    pub_goal_to_reach.publish(base_position);
    if (translation_to_base < translation_epsilon)
        current_state = resetting_orientation;
}

void process_resetting_orientation()
{

    if (state_has_changed)
    {
        ROS_WARN("current_state: initializing_rotation");
        ROS_WARN("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
        frequency = 0;
    }

    // Processing of the state
    // Robair rotates to its initial orientation
    // if robair is close to its initial orientation and does not move, after a while (use frequency), we switch to the state "waiting_for_a_person"
    if (new_localization)
    {
        ROS_WARN("position of robair in the map: (%f, %f, %f)", current_position.x, current_position.y, current_orientation * 180 / M_PI);
        frequency = 0;
    }
    else
        frequency++;

    // TODO: remember rotation!
    std_msgs::Float32 rotation_to_base_msg;
    rotation_to_base_msg.data = rotation_to_base;
    pub_rotation_to_do.publish(rotation_to_base_msg);
    if (frequency >= frequency_expected && rotation_to_base < rotation_epsilon)
        current_state = waiting_for_a_person;
}

// CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void person_positionCallback(const geometry_msgs::Point::ConstPtr &g)
{
    // process the goal received from moving_persons detector

    new_person_position = true;
    person_position.x = g->x;
    person_position.y = g->y;
    person_position.z = g->z;
    ROS_WARN("Person position: (%lf, %lf, %lf)", g->x, g->y, g->z);
}

void robot_movingCallback(const std_msgs::Bool::ConstPtr &state)
{

    robot_moving = state->data;

} // robot_movingCallback

void localizationCallback(const geometry_msgs::Point::ConstPtr &l)
{
    // process the localization received from my localization

    if (!init_localization)
    {
        base_position = *l;
        base_orientation = l->z;
    }

    new_localization = true;
    init_localization = true;
    current_position = *l;
    current_orientation = l->z;
}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb)
{

    return sqrt(pow((pa.x - pb.x), 2.0) + pow((pa.y - pb.y), 2.0));
}
}
;

int main(int argc, char **argv)
{

    ROS_INFO("(decision_node) waiting for a /person_position");
    ros::init(argc, argv, "decision_node");

    decision_node bsObject;

    ros::spin();

    return 0;
}
