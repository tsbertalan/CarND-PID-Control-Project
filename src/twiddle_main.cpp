#include <iostream>
#include <cmath>
#include <algorithm>  // std::min, std::max
#include <fstream> // std::ofstream
#include "json.hpp"

#include <uWS/uWS.h>
#include "PID.h"

#include "twiddle.h"
#include "say_time.h"


// Set parameters.
#define TARGETSPEED 40.0
#define CREEPSPEED 3.0
#define MAXANGLE 25.0
#define NSAMPLES 1600
#define NDISCARD 32
#define TWIDDLETOL 0.001

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}



int main() {
    uWS::Hub h;

    PID pid_steering;
    PID pid_throttle;

    // Need to tune PID parameters.

    // Manually chosen:
    // pid_steering.Init(0.2, 0.001, 4);

    // Bad initial values for twiddling:
    // pid_steering.Init(8e-2, 1e-4, 5e-1);

    // Pretty good values:
    // From a few twiddle runs:
    // pid_steering.Init(0.25409, 0.000654, 1.95139);
    // pid_steering.Init(0.117037, 0.000174074, 0.494455)
    // pid_steering.Init(0.320324, 0.00199993, 4.00688);

    // For Ziegler-Nichols:
    // pid_steering.Init(.1, 0, 0);
    // Gives: Kc = .05; Tc* = 1635 [ms]; Tc=Tc*/per_PID
    // Therefore, using Kp=0.6*Kc; Ki=2./Tc; Kd=8./Tc
    // Where per_PID=49 [ms] is the PID sampling period, I get
    // pid_steering.Init(.03, .059939, 4.1709);
    // This is reasonable for the Kd term, but not for the other two.
    // Without being able to make a step change, it's hard to judge the critical Kp value "Kc" for ZN.
    // So, while the Ki and Kd values are decent, the value of .03 that it gives for Kp is poor.
    // Ki and Kd come from period estimates, which I just took from the many recordings
    // of undamped oscillation I have. Kp, I chose manually as 0.2.
    // The Ki value is just too large--I'm not sure why.
    // Finally, after examining the PV(t), CV(t) recordings, I guessed that the large Kd value
    // was causing some of the overreacting to small disturbances, and so reduced it from the ZN prediction.
    // Basically, I took very little from ZN.
    pid_steering.Init(0.1, 0.001, 0.8);

    pid_throttle.Init(0.3, 0, 0.02);

    std::vector<PID*> pids = {&pid_steering};//, &pid_throttle};

    // Time-average the CTE to get an error value for Twiddle.
    TwiddlerManager twiddler_manager(pids, NSAMPLES, TWIDDLETOL, NDISCARD);

    std::ofstream cte_log_file;
    cte_log_file.open("cte.csv", std::ios::trunc);

    h.onMessage([&pid_steering, &pid_throttle, &twiddler_manager, &cte_log_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(std::string(data).substr(0, length));
            if (s != "") {
                auto j = json::parse(s);
                std::string event = j[0].get<std::string>();
                if (event == "telemetry") {
                    // j[1] is the data JSON object
                    double cte = std::stod(j[1]["cte"].get<std::string>());
                    double speed = std::stod(j[1]["speed"].get<std::string>());
                    double angle = std::stod(j[1]["steering_angle"].get<std::string>());


                    pid_steering.UpdateError(cte);

                    double speedTarget = TARGETSPEED;
                    pid_throttle.UpdateError(speed - speedTarget);


                    /*
                    * Calcuate steering value here, remember the steering value is
                    * [-1, 1].
                    * NOTE: Feel free to play around with the throttle and speed. Maybe use
                    * another PID controller to control the speed!
                    */
                    double steer_value = std::max(-1.0, std::min(1.0, pid_steering.TotalError()));
                    double throttle = std::max(pid_throttle.TotalError(), 0.0);

                    // Save to log file.
                    cte_log_file <<epoch_time()<<", " <<cte<<","   <<speed<<"," <<angle<<",";
                    cte_log_file <<steer_value<<","   <<throttle<<"," <<pid_steering.i_error<< std::endl;

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle;
                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // std::cout << msg << std::endl;
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                    twiddler_manager.process_error(cte);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
