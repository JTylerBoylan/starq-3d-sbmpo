#ifndef MY_CUSTOM_MODEL_HPP_
#define MY_CUSTOM_MODEL_HPP_

#include <sbmpo/types/types.hpp>
#include <sbmpo/types/Model.hpp>

#include <my_project/SDF3D.hpp>

#include <math.h>

#define OBSTACLE_MIN_DISTANCE 0.15F
#define GOAL_THRESHOLD 0.1F

#define COST_OF_TRANSPORT_WALK 10.0F
#define COST_OF_TRANSPORT_SWIM 100.0F
#define COST_TO_JUMP 100.0F

#define VELOCITY_WALK 0.05F
#define VELOCITY_WALK_DIAGONAL (VELOCITY_WALK * M_SQRT2 / 2.0)
#define VELOCITY_SWIM 0.1F
#define VELOCITY_SWIM_DIAGONAL (VELOCITY_SWIM * M_SQRT2 / 2.0)
#define JUMP_HEIGHT 0.25F

namespace my_namespace
{

    using namespace sbmpo;

    class MyCustomModel : public Model
    {

    public:
        // States of the Model
        enum States
        {
            X,
            Y,
            Z,
            Gait
        };

        // Controls of the Model
        enum Controls
        {
            dXdt,
            dYdt,
            dZdt,
            ControlGait
        };

        enum Gaits
        {
            Walk,
            Swim,
            Jump,
            Sink
        };

        // Constructor
        MyCustomModel()
        {
        }

        // Evaluate a node with a control
        State next_state(const State &state, const Control &control, const float time_span) override
        {
            /*
                Dynamics of the system
                How does each state change with respect to the controls?
            */

           const uint8_t gait = static_cast<uint8_t>(control[ControlGait]);

            State next_state = state;
            if (gait == Gaits::Jump)
            {
                next_state[Z] += JUMP_HEIGHT;
                next_state[Gait] = static_cast<float>(Gaits::Swim);
                return next_state;
            }
            else if (gait == Gaits::Sink)
            {
                next_state[Z] = 0.0;
                next_state[Gait] = static_cast<float>(Gaits::Walk);
                return next_state;
            }
            else
            {
                next_state[X] += control[dXdt] * time_span;
                next_state[Y] += control[dYdt] * time_span;
                next_state[Z] += control[dZdt] * time_span;
                next_state[Gait] = control[ControlGait];
            }
            return next_state;
        }

        // Get the cost of a control
        float cost(const State &state1, const State &state2, const Control &control, const float time_span) override
        {
            /*
                Cost of a state and control
                What am I trying to minimize?
                i.e Distance, Time, Energy
            */

            // return distance_cost(state1, state2, control, time_span);
            return energy_cost(state1, state2, control, time_span);
        }

        // Get the cost of a control
        float distance_cost(const State &state1, const State &state2, const Control &control, const float time_span)
        {
            const uint8_t gait = static_cast<uint8_t>(control[ControlGait]);
            const float swim_factor = gait == Gaits::Swim ? 1.01 : 1.0;
            
            const float dX = state2[X] - state1[X];
            const float dY = state2[Y] - state1[Y];
            const float dZ = state2[Z] - state1[Z];
            return swim_factor * sqrtf(dX * dX + dY * dY + dZ * dZ);
        }

        // Get the cost of a control
        float energy_cost(const State &state1, const State &state2, const Control &control, const float time_span)
        {
            const uint8_t gait = static_cast<uint8_t>(control[ControlGait]);
            switch (gait)
            {
            case Gaits::Walk:
                return COST_OF_TRANSPORT_WALK * time_span;
            case Gaits::Swim:
                return COST_OF_TRANSPORT_SWIM * time_span;
            case Gaits::Jump:
                return COST_TO_JUMP;
            }

            return 0.0;
        }

        // Get the heuristic of a state
        float heuristic(const State &state, const State &goal) override
        {
            /*
                Heuristic of a state with respect to the goal
                Leads the planner to the goal
                What is the lowest cost possible from this state to the goal?
            */

            const float dX = goal[X] - state[X];
            const float dY = goal[Y] - state[Y];
            const float dZ = goal[Z] - state[Z];
            return sqrtf(dX * dX + dY * dY + dZ * dZ);
        }

        // Determine if node is valid
        bool is_valid(const State &state) override
        {
            /*
                Does this state meet the model constraints?
                i.e Boundary constraints, Obstacles, State limits
            */

            float distance = sdf_.getDistance(state[X], state[Y], state[Z]);

            return distance >= OBSTACLE_MIN_DISTANCE &&
                   state[Z] >= 0.0;
        }

        // Determine if state is goal
        bool is_goal(const State &state, const State &goal) override
        {
            /*
                Is this state close enough to the goal to end the plan?
            */

            return this->heuristic(state, goal) <= GOAL_THRESHOLD;
        }

        std::vector<Control> getControlSamples(const State &state)
        {
            const uint8_t gait = static_cast<uint8_t>(state[Gait]);
            switch (gait)
            {
            case Gaits::Walk:
            {
                const float walk = state[Gait];
                return {{VELOCITY_WALK, 0.0, 0.0, walk},
                        {VELOCITY_WALK_DIAGONAL, VELOCITY_WALK_DIAGONAL, 0.0, walk},
                        {0.0, VELOCITY_WALK, 0.0, walk},
                        {-VELOCITY_WALK_DIAGONAL, VELOCITY_WALK_DIAGONAL, 0.0, walk},
                        {-VELOCITY_WALK, 0.0, 0.0, walk},
                        {-VELOCITY_WALK_DIAGONAL, -VELOCITY_WALK_DIAGONAL, 0.0, walk},
                        {0.0, -VELOCITY_WALK, 0.0, walk},
                        {VELOCITY_WALK_DIAGONAL, -VELOCITY_WALK_DIAGONAL, 0.0, walk},
                        {0.0, 0.0, 0.0, static_cast<float>(Gaits::Jump)}};
            }
            case Gaits::Swim:
            {
                const float swim = state[Gait];
                return {{VELOCITY_SWIM, 0.0, 0.0, swim},
                        {-VELOCITY_SWIM, 0.0, 0.0, swim},
                        {VELOCITY_SWIM_DIAGONAL, 0.0, VELOCITY_SWIM_DIAGONAL, swim},
                        {-VELOCITY_SWIM_DIAGONAL, 0.0, VELOCITY_SWIM_DIAGONAL, swim},
                        {VELOCITY_SWIM_DIAGONAL, 0.0, -VELOCITY_SWIM_DIAGONAL, swim},
                        {-VELOCITY_SWIM_DIAGONAL, 0.0, -VELOCITY_SWIM_DIAGONAL, swim},
                        {0.0, 0.0, 0.0, static_cast<float>(Gaits::Sink)}};
            }
            }
            return {};
        }

        void setSDF3D(SDF3D &sdf)
        {
            sdf_ = sdf;
        }

        // Deconstructor
        ~MyCustomModel() {}

    protected:
        SDF3D sdf_;
    };

}

#endif