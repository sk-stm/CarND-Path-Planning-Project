#include "Behavior.h"

#include "costs.h"
#include <functional>

Behavior::Behavior(Map const &map)
    : _map(map), _path_planner(map)
{
    // initial state (defined by the simulator)
    _state.wanted_lane = 1;
    _state.wanted_speed = 0.1;
    _state.maneuver = BehaviorState::KL;
}

Path Behavior::plan(CarState const &cs, Path const &previous_path, FrenetPoint end_point_frenet, Obstacles const &obstacles)
{
    using EvalTuple = std::tuple<BehaviorState, Path, double>;

    // find possible successor states
    auto possible_maneuvers = getPossibleManeuvers(cs);
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

    _visualizer.setCostData(evaluated_maneuvers);

    // execute and save maneuver
    auto const &best_maneuver = evaluated_maneuvers[0];
    _state = std::get<0>(best_maneuver);

    // set wanted speed (for next frame)
    double wanted_speed = calcWantedSpeedAcc(cs, obstacles);
    _visualizer.setDebugData(wanted_speed);
    assert(wanted_speed <= Map::MAX_LEGAL_SPEED);

    _state.wanted_speed = 0.95 * _state.wanted_speed + 0.05 * wanted_speed;

    return std::get<1>(best_maneuver);
}

double Behavior::calcWantedSpeedAcc(CarState const &cs, Obstacles const &obstacles) const
{
    auto closest_obst = obstacles.end();
    double closest_dist = std::numeric_limits<double>::max();

    for (auto obstIt = obstacles.begin(); obstIt < obstacles.end(); obstIt++)
    {

        if (obstIt->lane == cs.lane)
        {
            double distance = obstIt->s - cs.position_frenet.s;

            if (distance > 0 and distance < closest_dist)
            {
                closest_dist = distance;
                closest_obst = obstIt;
            }
        }
    }

    if (closest_obst == obstacles.end() or closest_dist > 30.)
    {
        return Map::MAX_LEGAL_SPEED;
    }

    if (closest_dist > 20)
    {
        return closest_obst->speed;
    }

    assert(closest_dist >= 0 and closest_dist <= 20);
    return closest_obst->speed * std::sqrt(closest_dist / 20.);
}

std::vector<BehaviorState> Behavior::getPossibleManeuvers(CarState const &cs) const
{
    std::vector<BehaviorState> possible_maneuvers;

    if (_state.maneuver == BehaviorState::KL)
    {
        BehaviorState s = _state;
        s.wanted_lane = cs.lane;
        s.maneuver = BehaviorState::KL;
        possible_maneuvers.push_back(s);

        size_t left_lane = _map.getLeftLaneOf(cs.lane);
        if (left_lane != cs.lane)
        {
            s.maneuver = BehaviorState::LCL;
            s.wanted_lane = left_lane;
            possible_maneuvers.push_back(s);
        }

        size_t right_lane = _map.getRightLaneOf(cs.lane);
        if (right_lane != cs.lane)
        {
            s.maneuver = BehaviorState::LCR;
            s.wanted_lane = right_lane;
            possible_maneuvers.push_back(s);
        }
    }
    else if (_state.maneuver == BehaviorState::LCL)
    {
        BehaviorState s = _state;
        s.maneuver = BehaviorState::KL;
        possible_maneuvers.push_back(s);
    }
    else if (_state.maneuver == BehaviorState::LCR)
    {
        BehaviorState s = _state;
        s.maneuver = BehaviorState::KL;
        possible_maneuvers.push_back(s);
    }

    return possible_maneuvers;
}

double Behavior::calculateCosts(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles)
{
    static const double INEFFICIENCY_WEIGHT{1};
    static const double SAFETY_WEIGHT{1};
    static const double RECHTSFAHRGEBOT_WEIGHT{1};
    static const double MAX_COST{1};
    static const double FREE_SPACE_AHEAD_WEIGHT{1};
    double cost = 0;

    std::vector<std::function<float(CarState const &cs, BehaviorState const &s, Path const &path, Obstacles const &obstacles, Map const &map)>> cf_list = {inefficiency_cost, safety_cost, rechtsfahrgebot_cost, free_space_ahead_cost};
    std::vector<double> weight_list = {INEFFICIENCY_WEIGHT, SAFETY_WEIGHT, RECHTSFAHRGEBOT_WEIGHT, FREE_SPACE_AHEAD_WEIGHT};
    double weight_sum = 0;

    for (int i = 0; i < cf_list.size(); i++)
    {
        double new_cost = cf_list[i](cs, s, path, obstacles, _map);
        assert(new_cost >= 0. and new_cost <= MAX_COST);

        // a cost of "1" means per protocoll, we should never do that maneuver (the cost-function prevents it)
        if (new_cost == MAX_COST)
        {
            return MAX_COST;
        }
        cost += weight_list[i] * new_cost;
        weight_sum += weight_list[i];
    }

    cost /= weight_sum;
    assert(cost >= 0. and cost <= MAX_COST);

    return cost;
}
