#include <rclcpp/rclcpp.hpp> // library that includes all the headers necessary to use the most common public pieces of the ROS2 system
#include <find_object_2d/ObjectsStamped.h>
#include "object_detector/Corners.h" // Custom msg of type Corners
#include "object_detector/States.h" // Custom msgs of type States

#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

# define M_PI   3.14159265358979323846  /* pi */

class States : public rclcpp::Node {
public:
	OffboardControl() : Node("cent_calculator") {
#ifdef ROS_DEFAULT_API
			// Publisher type object_detector::States, it publishes in /states topic
			pub = create_publisher<object_detector::States>( "/states", 10) ; 
#else
			// Publisher type object_detector::States, it publishes in /states topic
			pub = create_publisher<object_detector::States>( "/states", 10) ; 
		}
#endif
        sub =
			this->create_subscription<px4_msgs::msg::centCalculator>("/centCalculator", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

		// Subscriber callback
		void centCalculator(const object_detector::Corners& msg) //Callback para el subscriber "CentCalculator"
		{	
			// Creation of a States object to publish the info
			object_detector::States st;

			// Theta angle calculation
			float theta = computeTheta(msg.BottomLeftX, msg.BottomLeftY,
										msg.BottomRightX, msg.BottomRightY);
			// Bottom vector magnitude
			float v_bot = computeEucDist(msg.BottomLeftX, msg.BottomLeftY,
										msg.BottomRightX, msg.BottomRightY);
			// Right vector magnitude
			float v_right = computeEucDist(msg.BottomRightX, msg.BottomRightY,
										msg.TopRightX, msg.TopRightY);
			// Top vector magnitude
			float v_top = computeEucDist(msg.TopLeftX, msg.TopLeftY,
										msg.TopRightX, msg.TopRightY);
			// Left vector magnitude
			float v_left = computeEucDist(msg.TopLeftX, msg.TopLeftY,
										msg.BottomLeftX, msg.BottomLeftY);

			// Centroid of the template in the image
			st.Xc = msg.CenterX;
			st.Yc = msg.CenterY;
			// Computation of width and height as an average of the bottom and top magnitudes
			st.W = (v_bot + v_top) / 2; 
			st.H = (v_right + v_left) / 2;
			st.Theta = theta;

			// Correction of theta when the angle is bigger than 90 degrees to avoid bad orientation for the controller
			if(theta > 90) 
			{
				int cont = theta / 90;
				st.Theta = theta - cont * 90;
			}
			else if(theta < 90)
			{
				int cont = -1 * theta / 90;
				st.Theta = theta + cont * 90;
			}

			// Uncomment these lines to print the results on console
			/*printf("Centroid in pix position (%f,%f)\n Object Width (%f)\n Object Height (%f)\n Theta (%f)\n", 
							
					st.Xc, st.Yc,
					st.W, st.H,
					st.Theta);
			*/
			pub.publish(st);
		}

		/**
		 * Calculate the euclidian distance between two points
		 *
		 *
		 * @param x_1 coordiante x of first point
		 * @param x_2 coordiante x of first point
		 * @param x_3 coordiante x of first point
		 * @param x_4 coordiante x of first point
		 * @return norm euclidian dist of points
		 */
		float computeEucDist(const float x_1, const float y_1, const float x_2, const float y_2) 
		{
			float vector_x = x_2 - x_1;
			float vector_y = y_2 - y_1;
			return sqrt(abs(vector_x * vector_x) + abs(vector_y * vector_y));
		}

		/**
		 * Calculate the angle WRT X
		 *
		 *
		 * @param x_1 coordiante x of first point
		 * @param x_2 coordiante x of first point
		 * @param x_3 coordiante x of first point
		 * @param x_4 coordiante x of first point
		 * @return th angle WRT X
		 */
		float computeTheta(const float x_1, const float y_1, const float x_2, const float y_2) 
		{
			float vector_x = x_2 - x_1;
			float vector_y = y_2 - y_1;
			return atan2(vector_y, vector_x) * 180 / M_PI;
		}


int main(int argc, char** argv)
{   
    rclpp::init(argc, argv, "data_calculation"); // Node name
    rclpp::NodeHandle n;
    States compt(n);
    rclpp::spin();

    return 0;
}