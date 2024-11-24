#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;

#define MAX_THRUSTER_POS M_PI / 2

sensor_msgs::msg::NavSatFix::SharedPtr boat_data;
geometry_msgs::msg::PoseArray turbines_data;
sensor_msgs::msg::Imu imu_data;
std_msgs::msg::String ping;

double axe;
double distance;
float ajout;
int start_gps = 0;
const double EARTH_RADIUS = 6371000.0; // Earth's radius in meters
double distance_turbine = 1;           // 1 metre devant la turbone les moteurs s'arrete pour linstant
int around = 0;
int vitesse_max = 5000;
size_t nb_turbine_tmp = 0;
double lat_dest_deg;
double long_dest_deg;
const double EARTH_RADIUS_LAT = 111320.0; // Longueur d'un degré de latitude en mètres
const double DEG_TO_RAD = M_PI / 180.0;
int turbine_points;
float xdead;
float ydead;
int deadTurbine;

struct Point
{
  double x, y;
  Point(double x_val, double y_val) : x(x_val), y(y_val) {}
};

std::vector<Point> perimeter_points;
// Define a type for Circle (a vector of Points)
using Circle = std::vector<Point>;
using Rocks = std::vector<Point>;
std::vector<Circle> circles;
Circle fourPturbine;

// double radius = 20; // Radius of the circle champ magnetique de leolienne

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
class Navigation : public rclcpp::Node
{
public:
  Navigation() : Node("aquabot_node_cpp"),
                 m_thrust_left_pos(0),
                 m_thrust_right_pos(0),
                 m_thrust_right_power(0),
                 m_thrust_left_power(0),
                 windTurbines(0),
                 m_gps(0),
                 m_current_pos(0.0)
  {
    // Log that the node has succesfully started
    RCLCPP_INFO(this->get_logger(), "Hello world from aquabot_node_cpp!");

    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "gps_topic_mathys",
        10,
        std::bind(&Navigation::gps_topic_callback, this, std::placeholders::_1));

    subscription2_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "turbines_topic_mathys",
        10,
        std::bind(&Navigation::turbines_topic_callback, this, std::placeholders::_1));

    subscription3_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu_topic_mathys",
        10,
        std::bind(&Navigation::imu_topic_callback, this, std::placeholders::_1));

    sub_ping = this->create_subscription<std_msgs::msg::String>(
        "/vrx/windturbinesinspection/windturbine_checkup",
        10,
        std::bind(&Navigation::ping_topic_callback, this, std::placeholders::_1));

    // Create a publisher on the thusters position topics
    m_thrustersLeftTurbo = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/left/thrust", 10);
    m_thrustersRightTurbo = this->create_publisher<std_msgs::msg::Float64>("/aquabot/thrusters/right/thrust", 10);
    m_timer3 = this->create_wall_timer(0.01s, std::bind(&Navigation::thrust_power, this));
  }

private:
  float calculateThrust()
  {
    //The further we are from axe == 0, the more power when we rotate
      
    float angleDiff = std::fmod(std::abs(axe), 360.0f);

    // Ensure the angle difference is always between 0 and 180 degrees
    if (angleDiff > 180)
    {
      angleDiff = 360 - angleDiff;
    }

    // Calculate thrust based on the angle difference
    float powerFactor = (angleDiff / 180.0f); // Linear reduction

    // Calculate the current thrust power
    float currentThrust = 10000 * powerFactor;
    return currentThrust;
  }

  void publish_thrust_power()
  {
    auto left = std_msgs::msg::Float64();
    auto right = std_msgs::msg::Float64();
    left.data = m_thrust_left_power;
    right.data = m_thrust_right_power;
    m_thrustersRightTurbo->publish(left);
    m_thrustersLeftTurbo->publish(right);
  }

  void thrust_power()
  {

    if (turbines_data.poses.empty()) // Don't start intil we get data
      return;

    if (axe > 180)
    {
      m_thrust_right_power = calculateThrust();
      m_thrust_left_power = 0;
    }
    else
    {
      m_thrust_left_power = calculateThrust();
      m_thrust_right_power = 0;
    }

    if (axe > 359.0 || axe < 1.0)
    {
      ajout += 100;

      if (ajout > vitesse_max)
        ajout = vitesse_max;
    }
    else
    {
      ajout -= 100;
      if (ajout < 0)
        ajout = 0;
    }

    m_thrust_left_power += ajout;
    m_thrust_right_power += ajout;
    publish_thrust_power();
  }

  void calcuateDistance(float x, float y) const
  {
    double r = M_PI / 180;

    // Converting latitude and longitude values from degrees to radians
    float boat_x = boat_data->latitude * r;
    float boat_y = boat_data->longitude * r;
    float xx = x * r;
    float yy = y * r;

    // Calculating the distance between two points
    distance = sqrt(pow(xx - boat_x, 2) + pow(yy - boat_y, 2)) * 1000000;
  }

  void gps_topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) const
  {
    boat_data = msg;
  }

  void imu_topic_callback(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    imu_data = *msg;
  }

  void ping_topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    ping = *msg;
  }

  // Fonction pour convertir les degrés en radians
  double degToRad(double degrees) const
  {
    return degrees * M_PI / 180.0;
  }

  // Structure to represent a 3D point
  struct Point
  {
    double x, y, z;
  };

  struct Quaternion
  {
    double x, y, z, w;
  };

  struct Vector3
  {
    double x, y, z;
  };

  // Calcul de l'azimut entre deux points géographiques
  double calculateAzimut(double lat_boat_deg, double long_boat_deg, double lat_dest_deg, double long_dest_deg) const
  {
    double lat_boat = degToRad(lat_boat_deg);
    double lat_dest = degToRad(lat_dest_deg);
    double diff_longitude = degToRad(long_dest_deg) - degToRad(long_boat_deg);

    double angle_direction = atan2(
        sin(diff_longitude) * cos(lat_dest),
        cos(lat_boat) * sin(lat_dest) - sin(lat_boat) * cos(lat_dest) * cos(diff_longitude));

    double azimut_degrees = angle_direction * 180.0 / M_PI;

    // Normaliser l'azimut entre 0° et 360°
    if (azimut_degrees < 0)
      azimut_degrees += 360.0;
    return azimut_degrees;
  }

  // Fonction pour normaliser un vecteur
  Vector3 normalize(const Vector3 &v) const
  {
    double norm = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    return {v.x / norm, v.y / norm, v.z / norm};
  }
  // Convertir un quaternion en vecteur direction
  Vector3 quaternionToDirection(const Quaternion &q) const
  {
    // Le vecteur direction de la proue du bateau est (1, 0, 0)
    Vector3 v = {1, 0, 0}; // Direction avant du bateau (axe X)

    // Calcul de la direction après rotation par le quaternion
    double qx = q.x, qy = q.y, qz = q.z, qw = q.w;

    double ix = qw * v.x + qy * v.z - qz * v.y;
    double iy = qw * v.y + qz * v.x - qx * v.z;
    double iz = qw * v.z + qx * v.y - qy * v.x;
    double iw = -qx * v.x - qy * v.y - qz * v.z;

    // Le vecteur résultant
    Vector3 result = {
        ix * qw + iw * -qx + iy * -qz - iz * -qy,
        iy * qw + iw * -qy + iz * -qx - ix * -qz,
        iz * qw + iw * -qz + ix * -qy - iy * -qx};

    return normalize(result);
  }

  // Calcul de l'azimut du bateau à partir de la direction
  double calculateBoatAzimut(const Vector3 &direction) const
  {
    double azimut = atan2(direction.x, direction.y);
    azimut = azimut * 180.0 / M_PI;

    if (azimut < 0)
    {
      azimut += 360.0; // Normaliser entre 0° et 360°
    }

    return azimut;
  }

  // Function to get points along the perimeter of the circle
  Circle getCirclePerimeter(double x_center, double y_center, double radius, int num_points = 8) const
  {
    Circle cercle;
    radius = radius / 100000;
    for (int i = 0; i < num_points; ++i)
    {
      double t = (2 * M_PI * i) / num_points; // Parameter t ranges from 0 to 2pi
      double x = x_center + radius * cos(t);  // X coordinate on the circle perimeter
      double y = y_center + radius * sin(t);  // Y coordinate on the circle perimeter
      cercle.push_back({x, y});
    }
    return cercle;
  }

  int turn_around_turbine() const
  {
    perimeter_points = getCirclePerimeter(turbines_data.poses[nb_turbine_tmp].position.x, turbines_data.poses[nb_turbine_tmp].position.y, 18);

    auto min_iter = perimeter_points.begin();
    double min_distance = std::numeric_limits<double>::infinity();

    // Loop through the perimeter_points to calculate the distance
    for (auto it = perimeter_points.begin(); it != perimeter_points.end(); ++it)
    {
      // Calculate the distance using the formula
      double distance = sqrt(pow(it->x - boat_data->latitude, 2) + pow(it->y - boat_data->longitude, 2));

      // Check if the current distance is smaller than the minimum distance found so far
      if (distance < min_distance)
      {
        min_distance = distance;
        min_iter = it; // Update the iterator
        turbine_points = std::distance(perimeter_points.begin(), min_iter);
      }
    }

    lat_dest_deg = perimeter_points[turbine_points].x;
    long_dest_deg = perimeter_points[turbine_points].y;
    return 0;
  }

  // Function to check if a point is inside or outside the circle
  bool isInsideCircle(double x, double y, double radiusSquared, float obstaclex, float obstacley) const
  {
    // Calculate the squared distance from the point to the center
    double distanceSquared = sqrt(pow(obstaclex - x, 2) + pow(obstacley - y, 2)) * 1000000;
    return distanceSquared <= radiusSquared; // If distance is less than or equal to radius, it's inside
  }

  void front_line(double boatAzimut) const
  {
    for (int i = 1; i <= distance * 5; ++i)
    {
      // Convertir l'azimut en radians
      double azimuthRad = degToRad(boatAzimut);
      // Convertir la latitude et la longitude en radians
      double lat0Rad = degToRad(boat_data->latitude);
      // Calcul des changements de latitude et longitude en mètres
      double deltaLat = i * cos(azimuthRad) / EARTH_RADIUS;
      double deltaLon = i * sin(azimuthRad) / (EARTH_RADIUS * cos(lat0Rad));
      // Calcul des nouvelles coordonnées GPS
      double lat = boat_data->latitude + (deltaLat * 180.0 / M_PI);
      double lon = boat_data->longitude + (deltaLon * 180.0 / M_PI);

      // boucle pour savoir si je suis dans la zone 15*15 d'une turbine
      for (size_t i = 0; i < turbines_data.poses.size(); ++i)
      {                                                                                                                      
        if (isInsideCircle(lat, lon, 15 * 15, turbines_data.poses[i].position.x, turbines_data.poses[i].position.y) == true)
        {
          turn_around_turbine();
          return;
        }
      }
    }
  }

  void turbines_topic_callback(const geometry_msgs::msg::PoseArray msg) const
  {
    turbines_data = msg;

    if (ping.data.length() > 0)
    {
      std::size_t found = ping.data.find("KO");
      if (found != std::string::npos)
      {
        xdead = boat_data->latitude;
        ydead = boat_data->longitude;
        deadTurbine = nb_turbine_tmp;
      }
      if (nb_turbine_tmp == 0 && distance <= 10)
      {
        RCLCPP_INFO(this->get_logger(), "QR code dectected :(%s)", ping.data.c_str());
        nb_turbine_tmp = 1;
      }
      else if (nb_turbine_tmp == 1 && distance <= 10)
      {
        RCLCPP_INFO(this->get_logger(), "QR code dectected :(%s)", ping.data.c_str());
        nb_turbine_tmp = 2;
      }
      else if (nb_turbine_tmp == 2 && distance <= 10)
      {
        RCLCPP_INFO(this->get_logger(), "QR code dectected :(%s)", ping.data.c_str());
        nb_turbine_tmp = 3;
      }
      else if (nb_turbine_tmp == 3)
      {
        RCLCPP_INFO(this->get_logger(), "FINISHHHHHHH");
        lat_dest_deg = xdead;
        long_dest_deg = ydead;
      }
      ping.data = "";
      around = 0;
    }
    if (around == 0 && nb_turbine_tmp != 3)
    {
      turn_around_turbine();
      around++;
    }

    if (distance && distance < 1.5)
    {
      turbine_points++;
      if (static_cast<std::vector<Point>::size_type>(turbine_points) == perimeter_points.size())
        turbine_points = 0;

      lat_dest_deg = perimeter_points[turbine_points].x;
      long_dest_deg = perimeter_points[turbine_points].y;
    }

    // Calcul de l'azimut de la cible (éolienne)
    double targetAzimut = calculateAzimut(boat_data->latitude, boat_data->longitude, lat_dest_deg, long_dest_deg);

    // Exemple d'orientation du bateau (quaternion récupéré de l'IMU)
    Quaternion boatOrientation = {imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w}; // Quaternion pour une rotation de 90° autour de l'axe Y

    // Convertir l'orientation du bateau en direction (vecteur)
    Vector3 boatDirection = quaternionToDirection(boatOrientation);
    // jai ou je veux aller, je fais x

    // Calculer l'azimut du bateau à partir de sa direction
    double boatAzimut = calculateBoatAzimut(boatDirection);

    axe = boatAzimut - targetAzimut;
    if (axe < 0)
      axe = 360 + axe;
    else if (axe > 360)
      axe -= 360;

    calcuateDistance(lat_dest_deg, long_dest_deg);
    if (around == 1 && (axe > 359.5 || axe < 0.5))
    {
      // quand je suis face a l'eolienne, je regarde si il  y a des obstacles
      front_line(boatAzimut);
      around++;
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersRightTurbo;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr m_thrustersLeftTurbo;

  rclcpp::TimerBase::SharedPtr m_timer3;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription2_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription3_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ping;

  float m_thrust_left_pos;
  float m_thrust_right_pos;
  float m_thrust_right_power;
  float m_thrust_left_power;
  float windTurbines;
  float m_gps;
  float m_current_pos;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Navigation>());
  rclcpp::shutdown();
  return 0;
}
