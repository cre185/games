#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        masses = std::vector<Mass *>{};
        springs = std::vector<Spring *>{};
        masses.push_back(new Mass(start, node_mass, 0));
        Vector2D delta = (end-start)/(num_nodes-1);
        for(int i=1;i<num_nodes;i++){
            Vector2D current = start+i*delta;
            masses.push_back(new Mass(current, node_mass, 0));
            springs.push_back(new Spring(masses[i-1], masses[i], k));
        }
        // Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            float dist = (s->m1->position-s->m2->position).norm();
            Vector2D dir = (s->m1->position-s->m2->position)/dist; // b to a
            Vector2D force = s->k*dir*(dist-s->rest_length);
            s->m1->forces-=force;
            s->m2->forces+=force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // TODO (Part 2): Add global damping
                float kd = 0.005;
                auto acceleration = m->forces / m->mass + gravity - kd * m->velocity / m->mass;
                m->velocity += acceleration * delta_t;
                m->position += m->velocity * delta_t;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            float dist = (s->m1->position-s->m2->position).norm();
            Vector2D dir = (s->m1->position-s->m2->position)/dist; // b to a
            Vector2D force = s->k*dir*(dist-s->rest_length);
            s->m1->forces-=force;
            s->m2->forces+=force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                // TODO (Part 4): Add global Verlet damping
                auto acceleration = m->forces / m->mass + gravity;
                float damping_factor = 0.00005;
                m->position = temp_position+(1-damping_factor)*(temp_position-m->last_position)+acceleration*delta_t*delta_t;
                m->last_position = temp_position;
            }
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
