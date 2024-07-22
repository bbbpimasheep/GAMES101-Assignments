#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

const float k_damp = 0.00005f;

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        masses = vector<Mass*>(num_nodes);
        springs = vector<Spring*>(num_nodes - 1);

        Vector2D segment = (end - start) / (num_nodes - 1);
        for (int i = 0; i < num_nodes; i += 1) {
            Vector2D position = start + segment * i;
            masses[i] = new Mass(position, node_mass, false);
        }
        for (int i = 0; i < num_nodes - 1; i += 1) {
            springs[i] = new Spring(masses[i], masses[i + 1], k);
        }

//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D len = s->m2->position - s->m1->position,
                     dir = len / len.norm();
            s->m1->forces += s->k * (len.norm() - s->rest_length) * dir;
            s->m2->forces += s->k * (len.norm() - s->rest_length) * -dir;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                Vector2D force = m->forces + m->mass * gravity,
                         acc = force / m->mass;
                m->velocity += acc * delta_t;
                m->position += (m->velocity + acc * delta_t) * delta_t;
                // TODO (Part 2): Add global damping
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
            Vector2D len = s->m2->position - s->m1->position,
                     dir = len / len.norm();
            if (s->m1->pinned && !s->m2->pinned) {
                s->m2->position += (len.norm() - s->rest_length) * -dir;
            } else if (!s->m1->pinned && s->m2->pinned) {
                s->m1->position += (len.norm() - s->rest_length) * dir;
            } else if (!s->m1->pinned && !s->m2->pinned) {
                s->m1->position += (len.norm() - s->rest_length) * dir * 0.5f;
                s->m2->position += (len.norm() - s->rest_length) * -dir * 0.5f;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D force = m->forces + m->mass * gravity,
                         acc = force / m->mass;
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                Vector2D mem = m->position;
                m->position += (1 - k_damp) * (m->position - m->last_position) + acc * delta_t * delta_t;
                m->last_position = mem;
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
