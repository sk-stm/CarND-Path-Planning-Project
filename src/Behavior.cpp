#include "Behavior.h"

#include "costs.h"
#include <functional>

Behavior::Behavior(Map const &map)
    : _map(map), _path_planner(map)
{
    // initial state (defined by the simulator)
    _state.current_lane = 1;
    _state.wanted_lane = _state.current_lane;
    _state.wanted_speed = Map::MAX_LEGAL_SPEED;
    _state.maneuver = BehaviorState::KL;
}

Path Behavior::plan(CarState const &cs, Path const &previous_path, FrenetPoint end_point_frenet, Obstacles const &obstacles)
{
    using EvalTuple = std::tuple<BehaviorState, Path, double>;
    // update current lane
    int const new_current_lane = std::floor(cs.position_frenet.d / Map::LANE_WIDTH);
    _state.current_lane = new_current_lane;

    // find possible successor states
    auto possible_maneuvers = getPossibleManeuvers();
    assert(not possible_maneuvers.empty());
    LOGGER->info("Got '{}' possible maneuvers", possible_maneuvers.size());

    // generate paths for each possible successor state and evaluate it
    std::vector<EvalTuple> evaluated_maneuvers;
    for (auto const &m : possible_maneuvers)
    {
        auto path = _path_planner.plan(cs, previous_path, m);
        auto cost = calculateCosts(cs, m, path, obstacles);

        evaluated_maneuvers.emplace_back(std::move(m), std::move(path), cost);

        LOGGER->info("Evaluated possible manuever '{}' with cost {}", m.to_string(), cost);
    }

    // find the minimal cost maneuver
    std::sort(evaluated_maneuvers.begin(), evaluated_maneuvers.end(), [](EvalTuple const &a, EvalTuple const &b) -> bool {
        return std::get<2>(a) < std::get<2>(b);
    });

    _visualizer.setData(evaluated_maneuvers);

    // execute and save maneuver
    auto const &best_maneuver = evaluated_maneuvers[0];
    _state = std::get<0>(best_maneuver);

    // set speed
    std::pair<double, double> our_lane_d_range = std::make_pair(Map::LANE_WIDTH * _state.current_lane, Map::LANE_WIDTH * _state.current_lane + Map::LANE_WIDTH);
    bool is_obst_ahead_too_close = false;

    for (auto obst : obstacles)
    {
        bool const is_obst_on_my_lane = (obst.d >= our_lane_d_range.first) and (obst.d < our_lane_d_range.second);

        if (is_obst_on_my_lane)
        {
            // walkthrough magic:
            obst.s += (double)previous_path.size() * SECONDS_PER_SAMPLING * obst.speed;

            double const s_walkthrough_magic = (previous_path.empty()) ? cs.position_frenet.s : end_point_frenet.s;
            double const distance_walkthrough_magic = 30;
            is_obst_ahead_too_close = (obst.s > s_walkthrough_magic) and (obst.s < s_walkthrough_magic + distance_walkthrough_magic);

            if (is_obst_ahead_too_close)
            {
                break;
            }
        }
    }

    if (is_obst_ahead_too_close)
    {
        _state.wanted_speed -= 0.44704;
    }
    else
    {
        _state.wanted_speed += 0.44704;
    }

    _state.wanted_speed = std::clamp(_state.wanted_speed, 0., Map::MAX_LEGAL_SPEED);

    return std::get<1>(best_maneuver);
}

std::vector<BehaviorState> Behavior::getPossibleManeuvers()
{
    std::vector<BehaviorState> possibleManeuvers;

    if (_state.maneuver == BehaviorState::KL)
    {
        BehaviorState s = _state;
        s.current_lane = _state.current_lane;
        s.wanted_lane = _state.current_lane;
        s.maneuver = BehaviorState::KL;
        possibleManeuvers.push_back(s);

        size_t left_lane = _map.getLeftLaneOf(_state.current_lane);
        if (left_lane != _state.current_lane)
        {
            s.maneuver = BehaviorState::LCL;
            s.wanted_lane = left_lane;
            possibleManeuvers.push_back(s);
        }

        size_t right_lane = _map.getRightLaneOf(_state.current_lane);
        if (right_lane != _state.current_lane)
        {
            s.maneuver = BehaviorState::LCR;
            s.wanted_lane = right_lane;
            possibleManeuvers.push_back(s);
        }
    }
    else if (_state.maneuver == BehaviorState::LCL)
    {
        BehaviorState s = _state;
        s.maneuver = BehaviorState::KL;
        possibleManeuvers.push_back(s);
    }
    else if (_state.maneuver == BehaviorState::LCR)
    {
        BehaviorState s = _state;
        s.maneuver = BehaviorState::KL;
        possibleManeuvers.push_back(s);
    }

    return possibleManeuvers;
}

double Behavior::calculateCosts(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles)
{
    static const double INEFFICIENCY_WEIGHT{1};
    static const double SAFETY_WEIGHT{1};
    double cost = 0;

    std::vector<std::function<float(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles)>> cf_list = {inefficiency_cost, safety_cost};

    std::vector<double> weight_list = {INEFFICIENCY_WEIGHT, SAFETY_WEIGHT};
    double weight_sum = 0;

    for (int i = 0; i < cf_list.size(); i++)
    {
        double new_cost = cf_list[i](cs, s, path, obstacles, _map);
        assert(new_cost >= 0. and new_cost <= 1.);
        cost += weight_list[i] * new_cost;
        weight_sum += weight_list[i];
    }

    cost /= weight_sum;

    return cost;
}
