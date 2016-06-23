// -*- C++ -*- (c) 2016 Jan Mrázek <email@honzamrazek.cz>

#pragma once

#include <iostream>
#include <vector>
#include <array>
#include <algorithm>
#include <atoms/numeric/vector.h>
#include <atoms/type/tagged.h>

namespace yaqwsx {

// Plans a smooth trajectory for 2 wheeled robot among control points
// Double = base type for calculations
// Tag = additional information asociated with each control point segment
template <class Double, class Tag = std::tuple<>>
class PathPlannerBase {
public:
    using Position = atoms::Tagged<atoms::Vector2D<Double>, Tag,
                            atoms::KeepLeftMerge<Tag>>;
    using WheelPosition = atoms::Tagged<std::pair<Double, Double>, Tag>;
    using Path = std::vector<Position>;

    // Path planning parameters, default values are provided
    struct Params {
        Double time_step;          // Time steps of the robot - point each step
        Double dist_step;          // Granularity of path shape generation
        Double path_alpha;         // <0;1> - "stick to control points" ratio
        Double path_beta;          // <0;1> - "feel free to go away from control poitns"
        Double robot_width;        // Width of the robot wheels
        Double speed_alpha;        // <0;1> amount of a precision speed follows hard limits with
        Double speed_beta;         // <0;1> how rounded will the speed be?
        Double max_speed;          // Hard limit on robot speed (including wheels)
        Double max_acceleration;   // Hard limit on acceleration (including wheels)
        Double final_acc_time;     // Length of "soft start and end" (reduces initial jerk)
        unsigned speed_step_mult;  // Each speed_step_mult point is speed limit calculated
        unsigned traj_smooth_pass; // Number of smooth passes on trajecotry

        Params()
            : time_step(0.1), dist_step(2), path_alpha(0.7), path_beta(0.3),
              robot_width(30), speed_alpha(0.1), speed_beta(0.3), max_speed(20),
              max_acceleration(5), final_acc_time(0.25), speed_step_mult(10),
              traj_smooth_pass(3)
        {}
    };

    PathPlannerBase(Params p = Params())
        : m_params(p)
    {}

    void set_params(Params p) {
        m_params = p;
    }

    Path& control_points() {
        return m_control_points;
    }

    void compute() {
        Double distance_step = m_params.dist_step;
        m_path = m_control_points;
        for (unsigned i = m_params.traj_smooth_pass; i != 0; i--) {
            m_path = divide_by_steps(m_path, distance_step * i);
            m_path = smooth(m_path, m_params.path_alpha, m_params.path_beta, 0.001);
        }

        m_center_vel = speed(m_path, m_params.max_speed, m_params.max_acceleration,
            m_params.robot_width, m_params.time_step * 10);
        m_center_vel = smooth(m_center_vel, m_params.speed_alpha, m_params.speed_beta, 0.001);
        
        m_path = path_to_steps(m_path, m_center_vel, m_params.time_step * 10,
            m_params.final_acc_time, m_params.max_acceleration);

        for (unsigned i = 0; i != m_params.traj_smooth_pass - 1; i ++) {
            m_path = smooth(m_path, m_params.speed_alpha, m_params.speed_beta, 0.001);
        }
	    m_path = inject(m_path, m_params.speed_step_mult - 1);
	    for (unsigned i = 0; i != (m_params.speed_step_mult - 1) * m_params.traj_smooth_pass; i++) {
            m_path = smooth(m_path, m_params.speed_alpha, m_params.speed_beta, 0.001);
	    }

        m_left = offset(m_path, m_params.robot_width / 2, true);
        m_right = offset(m_path, m_params.robot_width / 2, false);

        m_center_vel = velocity(m_path, m_params.time_step);
        m_left_vel = velocity(m_left, m_params.time_step);
        m_right_vel = velocity(m_right, m_params.time_step);

        auto diff = m_path[1] - m_path[0];
        m_reconstructed = reconstruct(m_left_vel, m_right_vel, m_params.robot_width,
            m_path.front(), atan2(diff.y, diff.x));
    }

    const Path& get_path() {
        return m_path;
    }

    const Path& get_left() {
        return m_left;
    }

    const Path& get_right() {
        return m_right;
    }

    const Path& get_velocity() {
        return m_center_vel;
    }

    const Path& get_left_velocity() {
        return m_left_vel;
    }

    const Path& get_right_velocity() {
        return m_right_vel;
    }

    const Path& get_reconstructed() {
        return m_reconstructed;
    }

    std::vector<WheelPosition> left_right_dist() {
        std::vector<WheelPosition> res;
        res.reserve(m_left.size() + 1);
        res.push_back({0, 0});
        for (size_t i = 1; i != m_left.size(); i++) {
            auto prev = res.back();
            res.push_back({prev.first + (m_left[i] - m_left[i - 1]).length(),
                           prev.second + (m_right[i] - m_right[i - 1]).length() });
            res.back().tag = m_left[i].tag;
        }

        return res;
    }

private:
    Path m_control_points;
    Params m_params;

    Path m_path;
    Path m_left;
    Path m_right;

    Path m_center_vel;
    Path m_left_vel;
    Path m_right_vel;

    Path m_reconstructed;

    static Path decimate(Path p, unsigned rate) {
        size_t i = 0;
        for (size_t j = 0; j < p.size(); j += rate, i++) {
            p[i] = p[j];
        }
        p.resize(i);
        return p;
    }

    static Path inject(const Path& p) {
        Path res;
        for (size_t i = 0; i < p.size() - 1; i++) {
            res.push_back(p[i]);
            res.push_back((p[i] + p[i + 1]) / 2);
        }
        res.push_back(p.back());
        return res;
    }

    static Path inject(const Path& p, size_t num) {
        Path res;
        res.reserve(p.size() + num * (p.size() - 1));

        for (size_t i = 0; i < p.size() - 1; i++) {
            res.push_back(p[i]);

            for (size_t j = 1; j < num + 1; j++) {
                res.push_back((p[i + 1] - p[i]) * j / (num + 1) + p[i]);
            }
        }
        res.push_back(p.back());
        return res;
    }

    static Path divide_by_steps(const Path& p, Double step) {
        Path res;
        Double pos = 0;
        for (size_t i = 0; i < p.size() - 1; i++) {
            auto dir = p[i + 1] - p[i];
            Double len = dir.length();
            dir /= len;
            while(pos < len) {
                res.push_back(p[i] + dir * pos);
                pos += step;
            }
            pos -= len;
        }
        res.push_back(p.back());
        return res;
    }

    // Smoothes path
    static Path smooth(const Path& p, Double weigth_data, Double weigth_smooth,
        Double tolerance)
    {
        Path res = p;
        Double change;
        do {
            change = 0;
            for (size_t i = 1; i < p.size() - 1; i++) {
                Position orig = res[i];
                res[i] += weigth_data * (p[i] - res[i]) +
                          weigth_smooth * (res[i - 1] + res[i + 1] - 2 * res[i]);
                change += (orig - res[i]).length();
            }

        } while (change > tolerance);

        return res;
    }

    static std::vector<Double> distances(const Path& p) {
        std::vector<Double> res;
        for (size_t i = 1; i < p.size() - 1; i++) {
            res.push_back((p[i] - p[i-1]).length());
        }
        return res;
    }

    static std::vector<Double> distances_to_go(std::vector<Double> p) {
        for(size_t i = p.size() - 2; i > 0; i--) {
            p[i] += p[i + 1];
        }
        p[0] += p[1];
        return p;
    }

    static Path offset(const Path& p, Double offset, bool left) {
        std::vector<Double> gradient;
        for (size_t i = 0; i < p.size() - 1; i ++) {
            auto diff = p[i + 1] - p[i];
            gradient.push_back(atan2(diff.y, diff.x));
        }
        gradient.push_back(gradient.back());

        Path res;
        Double rot = left ? (M_PI / 2.0) : (- M_PI / 2.0); 
        for (size_t i = 0; i < gradient.size(); i++) {
            res.emplace_back(
                offset * cos(gradient[i] + rot) + p[i].x,
                offset * sin(gradient[i] + rot) + p[i].y);
            res.back().tag = p[i].tag;
        }
        return res;
    }

    static Double speed_limit(Double v, Double max_v, Double max_a, Double distance, Double distance_to_go) {
        Double max_v_acc = sqrt(2 * distance * max_a + v * v);
        Double max_v_decc = sqrt(2 * distance_to_go * max_a);

        return std::min({max_v, max_v_acc, max_v_decc});
    }

    static Path speed(const Path& p, Double max_v, Double max_a,
        Double width, Double time_step)
    {
        Path left = offset(p, width / 2.0, true);
        Path right = offset(p, width / 2.0, false);
        auto dist = distances(p);
        auto left_dist = distances(left);
        auto right_dist = distances(right);
        auto to_go = distances_to_go(dist);
        auto left_to_go = distances_to_go(left_dist);
        auto right_to_go = distances_to_go(right_dist);

        Path res;
        Double left_v = 0, right_v = 0, v = 0;
        res.emplace_back(0, 0);
        for (size_t i = 0; i != dist.size(); i++) {
            Double l_limit = speed_limit(left_v, max_v, max_a, left_dist[i], left_to_go[i]);
            Double r_limit = speed_limit(right_v, max_v, max_a, right_dist[i], right_to_go[i]);

            Double candidate = speed_limit(v, max_v, max_a, dist[i], to_go[i]);

            Double l_candidate = left_dist[i] / dist[i] * candidate;
            Double r_candidate = right_dist[i] / dist[i] * candidate;
            if (l_candidate > l_limit || r_candidate > r_limit) {
                if (l_candidate > l_limit && r_candidate <= r_limit) {
                    candidate = dist[i] / left_dist[i] * l_limit;
                }
                else if (l_candidate <= l_limit && r_candidate > r_limit) {
                    candidate = dist[i] / right_dist[i] * r_limit;
                }
                else if (l_candidate < r_candidate) {
                    candidate = dist[i] / left_dist[i] * l_limit;
                }
                else {
                    candidate = dist[i] / right_dist[i] * r_limit;
                }
            }

            res.emplace_back(i + 1, candidate);
            v = candidate;
            left_v = left_dist[i] / dist[i] * candidate;
            right_v = right_dist[i] / dist[i] * candidate;
        }
        res.emplace_back(p.size() + 1, 0);
        return res;
    }

    Path path_to_steps(const Path& p, Path v, Double time_step, Double acc_time,
        Double max_a)
    {
        Path res;
        auto dis = distances(p);

        for (Double i = 0; i <= acc_time; i += time_step)
            res.push_back(p.front());

        Double pos = 0;
        for (size_t i = 0; i != p.size() - 1; i++) {
            auto dir = p[i + 1] - p[i];
            Double len = dir.length();
            dir /= len;
            while(pos < len) {
                res.push_back(p[i] + dir * pos);
                Double a = std::max(0.0, std::min(max_a, (v[i+1].y - v[i].y) / time_step));
                pos += v[i].y * time_step + a * time_step * time_step / 2;
                v[i].y += a * time_step;
            }
            pos -= len;
        }
        
        for (Double i = 0; i <= acc_time; i += time_step)
            res.push_back(p.back());

        return res;
    }

    // Constructs a velocity from positions
    // Result contains two values - x is a time step, y is a speed
    Path velocity(const Path& p, Double time_step) {
        Path res;
        res.emplace_back(0, 0);
        for(size_t i = 1; i < p.size(); i++) {
            res.emplace_back(res.back().x + time_step,
                             ((p[i] - p[i - 1]) / time_step).length());
        }
        return res;
    }

    static Path reconstruct(const Path& left_vel, const Path& right_vel,
        Double robot_width, Position initial, Double orient)
    {
        Double step = left_vel[1].x - left_vel[0].x;
        Path res;
        res.push_back(initial);
        for (size_t i = 0; i != left_vel.size(); i++) {
            Double v = (left_vel[i].y + right_vel[i].y) / 2;
            Double w = (- left_vel[i].y + right_vel[i].y) / robot_width;

            res.emplace_back(res.back().x + v * step * cos(orient),
                             res.back().y + v * step * sin(orient));
            /*res.emplace_back(res.back().x + v * (sin(orient + step * w) - sin(orient)) / w,
                             res.back().y + v * (cos(orient) - cos(orient + step * w)) / w);*/
            orient += w * step;
        }
        return res;
    }
};

template <class Tag = std::tuple<>>
using PathPlanner = PathPlannerBase<double, Tag>;

template <class Tag = std::tuple<>>
using PathPlannerSingle = PathPlannerBase<float, Tag>;

} // namespace yaqwsx