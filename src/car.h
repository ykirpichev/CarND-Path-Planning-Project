#ifndef CAR_H
#define CAR_H


#include "json.hpp"

#include <vector>

class Car {
public:
    double car_x;
    double car_y;
    double car_s;
    double car_d;
    double car_yaw;
    double car_speed;
    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;
    double end_path_s;
    double end_path_d;
    std::vector<Car> sensor_fusion;
    double prev_size;

    double ref_x;
    double ref_y;
    double ref_yaw;

    explicit Car(const std::vector<double>& sensor_fusion);

    explicit Car(const nlohmann::json& jobj);

    bool isLaneSafe(int lane);

    double laneVelocity(int lane);


    std::pair<std::vector<double>, std::vector<double>> generateTrajectory(int ref_lane,
                                                                           double ref_velocity,
                                                                           const std::vector<double>& maps_s,
                                                                           const std::vector<double>& maps_x,
                                                                           const std::vector<double>& maps_y);
};

#endif // CAR_H
