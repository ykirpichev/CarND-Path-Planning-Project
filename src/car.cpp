#include "car.h"

#include "helper.h"
#include "spline.h"

using namespace std;

Car::Car(const std::vector<double>& sensor_fusion) {
    car_x = sensor_fusion[0];
    car_y = sensor_fusion[1];
    car_yaw = sensor_fusion[2];
    double vx = sensor_fusion[3];
    double vy = sensor_fusion[4];
    car_speed = sqrt(vx * vx + vy * vy);
    car_s = sensor_fusion[5];
    car_d = sensor_fusion[6];
}

Car::Car(const nlohmann::json::basic_json& jobj)
{
    car_x = jobj["x"];
    car_y = jobj["y"];
    car_s = jobj["s"];
    car_d = jobj["d"];
    car_yaw = jobj["yaw"];
    car_speed = jobj["speed"];

    // Previous path data given to the Planner
    previous_path_x.assign(jobj["previous_path_x"].begin(), jobj["previous_path_x"].end());
    previous_path_y.assign(jobj["previous_path_y"].begin(), jobj["previous_path_y"].end());
    // Previous path's end s and d values
    end_path_s = jobj["end_path_s"];
    end_path_d = jobj["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    for (auto& sf : jobj["sensor_fusion"]) {
        sensor_fusion.emplace_back(std::vector<double>(sf.begin(), sf.end()));
    }

    prev_size = previous_path_x.size();

    ref_x = car_x;
    ref_y = car_y;
    ref_yaw = deg2rad(car_yaw);
    if (prev_size >= 2)
    {
        ref_x = previous_path_x[prev_size - 1];
        double diff_x = ref_x - previous_path_x[prev_size - 2];
        ref_y = previous_path_y[prev_size - 1];
        double diff_y = ref_y - previous_path_y[prev_size - 2];
        ref_yaw = atan2(diff_y, diff_x);

        car_s = end_path_s;
        car_speed = sqrt(diff_x * diff_x + diff_y * diff_y) / TIME_INTERVAL * MPS_TO_MILESPH;
    }
    }

bool Car::isLaneSafe(int lane)
{
    bool lane_safe = true;
    for(int i = 0; i < sensor_fusion.size(); i++)
    {
        float d = sensor_fusion[i].car_d;
        if(d <  4 * (lane + 1) && d > 4 * lane)
        {
            double check_speed = sensor_fusion[i].car_speed;
            double check_car_s = sensor_fusion[i].car_s;

            check_car_s += TIME_INTERVAL * prev_size * check_speed;
            double dist_s = check_car_s - car_s;
            if(dist_s < 30 && dist_s > -15)
            {
                lane_safe = false;
                break;
            }
        }
    }

    return lane_safe;
}

double Car::laneVelocity(int lane)
{
    double ref_velocity = 49.5;
    double closest_dist = 100000;
    for (int i = 0; i < sensor_fusion.size(); ++i)
    {
        //car is in my lane
        float d = sensor_fusion[i].car_d;
        if (d > 4 * lane && d < 4 * lane + 4)
        {
            double check_speed = sensor_fusion[i].car_speed;
            double check_car_s = sensor_fusion[i].car_s;
            check_car_s += TIME_INTERVAL * prev_size * check_speed;

            if ((check_car_s > car_s) && ((check_car_s - car_s) < 30) &&
                ((check_car_s - car_s) < closest_dist))
            {
                closest_dist = check_car_s - car_s;

                if (check_car_s - car_s > 20)
                {
                    //match that cars speed
                    ref_velocity = check_speed * MPS_TO_MILESPH;
                }
                else
                {
                    //go slightly slower than the cars speed
                    ref_velocity = check_speed * MPS_TO_MILESPH - 5;
                }
            }
        }
    }
    return ref_velocity;
}


pair<vector<double>, vector<double>> Car::generateTrajectory(int ref_lane,
                                                             double ref_velocity,
                                                             const vector<double>& maps_s,
                                                             const vector<double>& maps_x,
                                                             const vector<double>& maps_y)
{
    int prev_size = previous_path_x.size();

    vector<double> next_x_vals;
    vector<double> next_y_vals;

    vector<double> points_x;
    vector<double> points_y;

    if (prev_size < 2) {
        double prev_car_x = car_x - cos(car_yaw);
        double prev_car_y = car_y - sin(car_yaw);

        points_x.push_back(prev_car_x);
        points_x.push_back(car_x);

        points_y.push_back(prev_car_y);
        points_y.push_back(car_y);

    } else {
        points_x.push_back(previous_path_x[prev_size - 2]);
        points_x.push_back(previous_path_x[prev_size - 1]);

        points_y.push_back(previous_path_y[prev_size - 2]);
        points_y.push_back(previous_path_y[prev_size - 1]);
    }

    for (int i = 30; i <= 90; i += 30) {
        auto next_wp = getXY(car_s + i, 2 + 4 * ref_lane, maps_s, maps_x, maps_y);

        points_x.push_back(next_wp[0]);
        points_y.push_back(next_wp[1]);
    }

    for (int i = 0; i < points_x.size(); ++i) {
        double diff_x = points_x[i] - ref_x;
        double diff_y = points_y[i] - ref_y;

        points_x[i] = diff_x * cos(-ref_yaw) - diff_y * sin(-ref_yaw);
        points_y[i] = diff_x * sin(-ref_yaw) + diff_y * cos(-ref_yaw);
    }

    tk::spline s;

    s.set_points(points_x, points_y);

    for (int i = 0; i < previous_path_x.size(); ++i) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30.0;
    double target_y = s(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;

    for (int i = 1; i <= 50 - previous_path_x.size(); ++i) {

        if(ref_velocity > car_speed)
        {
            car_speed += 0.1 * MPS_TO_MILESPH;
        }
        else if(ref_velocity < car_speed)
        {
            car_speed -= 0.1 * MPS_TO_MILESPH;
        }


        double N = target_dist / (TIME_INTERVAL * car_speed / MPS_TO_MILESPH);
        double x_point = x_add_on + target_x / N;
        double y_point = s(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }

    return {next_x_vals, next_y_vals};
}
