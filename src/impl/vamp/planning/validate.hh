#pragma once

#include <cstdint>

#include <vamp/utils.hh>
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/planning/bezier.hh>

namespace vamp::planning
{
    template <std::size_t n, std::size_t... I>
    inline constexpr auto generate_percents(std::index_sequence<I...>) -> std::array<float, n>
    {
        return {(static_cast<void>(I), static_cast<float>(I + 1) / static_cast<float>(n))...};
    }

    template <std::size_t n>
    struct Percents
    {
        inline static constexpr auto percents = generate_percents<n>(std::make_index_sequence<n>());
    };

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_vector(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &vector,
        float distance,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            block[i] = start.broadcast(i) + (vector.broadcast(i) * percents);
        }

        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
        if (not valid or n == 1)
        {
            return valid;
        }

        const auto backstep = vector / (rake * n);
        for (auto i = 1U; i < n; ++i)
        {
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                block[j] = block[j] - backstep.broadcast(j);
            }

            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
            if (not valid)
            {
                return false;
            }
        }

        return true;
    }

    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        auto vector = goal - start;
        return validate_vector<Robot, rake, resolution>(start, vector, vector.l2_norm(), environment);
    }

    // topple addons
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_bez(
        const typename Robot::Configuration &start,
        Bezier bez,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // TODO: Fix use of reinterpret_cast in pack() so that this can be constexpr
        const auto percents = FloatVector<rake>(Percents<rake>::percents);

        typename Robot::template ConfigurationBlock<rake> block;

        // HACK: broadcast() implicitly assumes that the rake is exactly VectorWidth
        // use percents as times to get configs
        std::vector<state> states_vec;
        for (int i = 0; i < rake; i++) {
            states_vec.push_back(bez.evaluate(percents[i]));
        }

        row_matrix states(rake, Robot::dimension);
        for (int i = 0; i < rake; i++) {
            states(i) = states_vec[i];
        }

        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            // block[i] contains ith joint of all configs in rake
            block[i] = states.transpose()(i);
        }

        const std::size_t n = std::max(std::ceil(distance / static_cast<float>(rake) * resolution), 1.F);

        bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                 Robot::template fkcc<rake>(environment, block);
        if (not valid or n == 1)
        {
            return valid;
        }

        // slide the rake back along bez (i.e. compute new timesteps to rake)
        const auto backstep = percents / (rake * n);
        for (auto i = 1U; i < n; ++i)
        {
            // evaluate states in rake
            auto times = percents - i * backstep;
            for (int j = 0; j < rake; j++) {
                states_vec.push_back(bez.evaluate(percents[j]));
            }
            // get matrix of states
            for (int j = 0; j < rake; j++) {
                states(j) = states_vec[j];
            }
            for (auto j = 0U; j < Robot::dimension; ++j)
            {
                // block[i] contains ith joint of all configs in rake
                block[j] = states.transpose()(j);
            }

            bool valid = (environment.attachments) ? Robot::template fkcc_attach<rake>(environment, block) :
                                                     Robot::template fkcc<rake>(environment, block);
            if (not valid)
            {
                return false;
            }
        }

        return true;
    }

    // only call this
    template <typename Robot, std::size_t rake, std::size_t resolution>
    inline constexpr auto validate_bez_motion(
        const typename Robot::Configuration &start,
        const typename Robot::Configuration &goal,
        const collision::Environment<FloatVector<rake>> &environment) -> bool
    {
        // build input to MLP
        std::vector<double> x;
        for (int i = 0; i < Robot::dimension; i++) {
            x.push_back(start[i])
        }
        for (int i = 0; i < Robot::dimension; i++) {
            x.push_back(goal[i])
        }

        // array to store inference output
        std::array<double, 29> out;
        Robot::template topple_nn_forward(x, out);

        // build the anchors
        row_matrix anchors(6, Robot::dimension);

        // initial point
        for (int i = 0; i < Robot::dimension; i++) {
            anchors(0, i) = start[i];
        }

        // intermediate points
        for (int i = 1; i <= 4; i++) {
            for (int j = 0; j < Robot::dimension; j++) {
                anchors(i, j) = out[(i - 1) * Robot::dimension + j];
            }
        }

        // final point
        for (int i = 0; i < Robot::dimension; i++) {
            anchors(5, i) = goal[i];
        }

        Bezier bez(anchors);
        return validate_bez<Robot, rake, resolution>(start, bez, environment);
    }    
}  // namespace vamp::planning
