#pragma once

#include "matplotlibcpp.h"
#include "utils.hpp"

#include <chrono>
#include <mutex>
#include <thread>
#include <tuple>

namespace plt = matplotlibcpp;

class Visualizer
{

  public:
    Visualizer()
    {

        data_[BehaviorState::Maneuver::KL] = std::vector<double>();
        data_[BehaviorState::Maneuver::LCL] = std::vector<double>();
        data_[BehaviorState::Maneuver::LCR] = std::vector<double>();

        run_ = true;

        // setup the worker thread
        worker_ = std::thread([&]() {
            while (run_)
            {
                if (dirty_)
                {
                    dirty_ = false;
                    plot();
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        });
    }

    ~Visualizer()
    {
        run_ = false;
        worker_.join();
    }

    void setData(std::vector<std::tuple<BehaviorState, Path, double>> const &data)
    {
        std::lock_guard<std::mutex> lock{mutex_};

        for (auto &kv : data_)
        {
            bool found = false;
            for (auto maneuver_data : data)
            {
                if (std::get<0>(maneuver_data).maneuver == kv.first)
                {
                    kv.second.push_back(std::get<2>(maneuver_data));
                    found = true;
                    break;
                }
            }

            if (not found)
            {
                kv.second.push_back(1.0);
            }

            while (kv.second.size() > 150)
            {
                kv.second.erase(kv.second.begin());
            }
        }

        dirty_ = true;
    }

  private:
    std::mutex mutex_;
    std::thread worker_;
    std::map<BehaviorState::Maneuver, std::vector<double>> data_;

    bool dirty_{false};
    bool run_{false};

    void plot()
    {
        mutex_.lock();

        // clear previous plot
        plt::clf();
        plt::xkcd();
        double max_y = 1.1;
        for (auto &kv : data_)
        {
            BehaviorState s;
            s.maneuver = kv.first;
            auto const &name = s.to_string();
            plt::named_plot(name.c_str(), kv.second);
            if (not kv.second.empty())
                max_y = std::max(max_y, *(std::max_element(kv.second.begin(), kv.second.end())));
        }
        plt::title("costs");
        plt::ylim(-0.1, max_y);
        plt::legend();

        mutex_.unlock();

        plt::pause(0.001);
    };
};