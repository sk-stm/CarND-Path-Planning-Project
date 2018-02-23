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

        cost_data_[BehaviorState::Maneuver::KL] = std::vector<double>();
        cost_data_[BehaviorState::Maneuver::LCL] = std::vector<double>();
        cost_data_[BehaviorState::Maneuver::LCR] = std::vector<double>();

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

    void setCostData(std::vector<std::tuple<BehaviorState, Path, double>> const &data)
    {
        std::lock_guard<std::mutex> lock{mutex_};

        for (auto &kv : cost_data_)
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

    void setDebugData(double const &d)
    {
        std::lock_guard<std::mutex> lock{mutex_};
        debug_data_.push_back(d);

        while (debug_data_.size() > 150)
        {
            debug_data_.erase(debug_data_.begin());
        }
    }

  private:
    std::mutex mutex_;
    std::thread worker_;
    std::map<BehaviorState::Maneuver, std::vector<double>> cost_data_;
    std::vector<double> debug_data_;

    bool dirty_{false};
    bool run_{false};

    void plot()
    {
        mutex_.lock();
        // copy data in order to release lock early
        auto data = cost_data_;
        mutex_.unlock();

        // clear previous plot
        plt::clf();

        // plot actions and their costs
        plt::subplot(2, 1, 1);
        plt::xkcd();
        double max_y = 1.1;
        for (auto &kv : data)
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

        // debug plot
        plt::subplot(2, 1, 2);
        plt::xkcd();
        plt::named_plot("current lane", debug_data_);
        plt::title("debug");
        plt::legend();

        plt::pause(0.001);
    };
};