#include "Behavior.h"
#include "Map.h"
#include "Obstacles.h"

#include "json.hpp"
#include "spdlog/spdlog.h"

#include <uWS/uWS.h>
#include <vector>

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s)
{
	auto found_null = s.find("null");
	auto b1 = s.find_first_of("[");
	auto b2 = s.find_first_of("}");
	if (found_null != std::string::npos)
	{
		return "";
	}
	else if (b1 != std::string::npos && b2 != std::string::npos)
	{
		return s.substr(b1, b2 - b1 + 2);
	}
	return "";
}

int main()
{
	// init logger
	auto console = spdlog::stdout_color_mt("console");
	spdlog::set_level(spdlog::level::debug);

	uWS::Hub h;

	Map map("../data/highway_map.csv");
	Behavior behavior(map);

	h.onMessage([&behavior](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
							uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{

			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);

				std::string event = j[0].get<std::string>();

				if (event == "telemetry")
				{
					// j[1] is the data JSON object

					// Main car's localization Data
					double car_x = j[1]["x"];
					double car_y = j[1]["y"];
					double car_s = j[1]["s"];
					double car_d = j[1]["d"];
					double car_yaw = j[1]["yaw"];
					double car_speed = double(j[1]["speed"]) * 0.44704; // convert units MPH -> m/s;

					// Previous path data given to the Planner
					std::vector<double> previous_path_x = j[1]["previous_path_x"];
					std::vector<double> previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					std::vector<std::vector<double>> sensor_fusion = j[1]["sensor_fusion"];

					json msgJson;

					Path previous_path(previous_path_x, previous_path_y);
					CarState cs(Point(car_x, car_y), deg2rad(car_yaw), FrenetPoint(car_s, car_d), car_speed);
					Obstacles obstacles(sensor_fusion);
					Path new_path = behavior.plan(cs, previous_path, FrenetPoint(end_path_s, end_path_d), obstacles);

					// convert the path back and send it
					std::vector<double> next_x_vals;
					std::vector<double> next_y_vals;
					std::tie(next_x_vals, next_y_vals) = new_path.split();
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";

					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
		}
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
					   size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						   char *message, size_t length) {
		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
	}
	else
	{
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}
	h.run();
}
