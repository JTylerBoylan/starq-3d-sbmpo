#ifndef MY_CUSTOM_MODEL_HPP_
#define MY_CUSTOM_MODEL_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Model.hpp>

#include <math.h>

namespace my_namespace {

using namespace sbmpo;

class MyCustomModel : public Model {

    public:

    // States of the Model
    enum States {X, Y};

    // Controls of the Model
    enum Controls {dXdt, dYdt};

    // Constructor
    MyCustomModel() {
        x_bounds_ = {-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
        y_bounds_ = {-std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity()};
        goal_threshold_ = 0.25f;
    }

    // Evaluate a node with a control
    State next_state(const State &state, const Control& control, const float time_span) override {

        /*
            Dynamics of the system
            How does each state change with respect to the controls?
        */

       State next_state = state;
       next_state[X] += control[dXdt] * time_span;
       next_state[Y] += control[dYdt] * time_span;
       return next_state;

    }

    // Get the cost of a control
    float cost(const State& state1, const State& state2, const Control& control, const float time_span) override {

        /*
            Cost of a state and control
            What am I trying to minimize?
            i.e Distance, Time, Energy
        */

        return sqrtf(control[dXdt]*control[dXdt] + control[dYdt]*control[dYdt])*time_span;
    }

    // Get the heuristic of a state
    float heuristic(const State& state, const State& goal) override {

        /*
            Heuristic of a state with respect to the goal
            Leads the planner to the goal
            What is the lowest cost possible from this state to the goal?
        */

        const float dX = goal[X] - state[X];
        const float dY = goal[Y] - state[Y];
        return sqrtf(dX*dX + dY*dY);
    }

    // Determine if node is valid
    bool is_valid(const State& state) override {

        /*
            Does this state meet the model constraints?
            i.e Boundary constraints, Obstacles, State limits
        */

        return state[X] > x_bounds_[0] &&
               state[X] < x_bounds_[1] &&
               state[Y] > y_bounds_[0] &&
               state[Y] < y_bounds_[1];
    }

    // Determine if state is goal
    bool is_goal(const State& state, const State& goal) override {

        /*
            Is this state close enough to the goal to end the plan?
        */
       
        return this->heuristic(state, goal) <= goal_threshold_;
    }

    // Deconstructor
    ~MyCustomModel() {}

    /// @brief Set the map bounds of the plan
    void set_bounds(float min_x, float max_x, float min_y, float max_y) {
        x_bounds_ = {min_x, max_x};
        y_bounds_ = {min_y, max_y};
    }

    /// @brief Set the goal threshold
    void set_goal_threshold(float goal_threshold) {
        goal_threshold_ = goal_threshold;
    }

    protected:

    std::array<float, 2> x_bounds_;
    std::array<float, 2> y_bounds_;
    float goal_threshold_;


};

}

#endif