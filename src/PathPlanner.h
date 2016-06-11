// -*- C++ -*- (c) 2016 Jan Mrázek <email@honzamrazek.cz>

#pragma once

#include <vector>
#include <array>
#include <atoms/numeric/vector.h>

namespace yaqwsx {

template <class Double>
class PathPlannerBase {
public:
    using Position = atoms::Vector2D<Double>;
    using Path = std::vector<Position>;

    struct Params {
        Double time_step;
        Double path_alpha;
        Double path_beta;
        Double robot_width;
        Double speed_alpha;
        Double speed_beta;

        Params(Double time_step = 0.1, Double robot_width = 20,
               Double path_alpha = 0.7, Double path_beta = 0.3,
               Double speed_alpha = 0.1, Double speed_beta = 0.3)
            : time_step(time_step), path_alpha(path_alpha), path_beta(path_beta),
              robot_width(robot_width), speed_alpha(speed_alpha),
              speed_beta(speed_beta)
        {}
    };

    PathPlannerBase(const Path& path, Params p = Params())
        : m_control_points(path), m_params(p)
    {}

    void compute() {
        auto ratios = split_ratio(m_control_points.size(), 10, m_params.time_step);
        m_path = m_control_points;
        for (int i : ratios) {
            m_path = inject(m_path, i);
            m_path = smooth(m_path, m_params.path_alpha, m_params.path_beta, 0.001);
        }

        left_right(m_params.robot_width);

        auto center_vel = velocity(m_path, m_params.time_step);
        auto left_vel = velocity(m_left, m_params.time_step);
        auto right_vel = velocity(m_right, m_params.time_step);

        m_center_vel = center_vel;
        m_left_vel = left_vel;
        m_right_vel = right_vel;

        m_center_vel.back().y = 0;
        m_left_vel.back().y = 0;
        m_right_vel.back().y = 0;

        for (int i = 0; i != 2; i++) {
            m_center_vel = smooth(m_center_vel, m_params.speed_alpha, m_params.speed_beta, 0.001);
            m_left_vel = smooth(m_left_vel, m_params.speed_alpha, m_params.speed_beta, 0.001);
            m_right_vel = smooth(m_right_vel, m_params.speed_alpha, m_params.speed_beta, 0.001);
        }

        m_center_vel = velocity_fix(m_center_vel, center_vel, 0.0001);
        m_left_vel = velocity_fix(m_left_vel, left_vel, 0.0001);
        m_right_vel = velocity_fix(m_right_vel, right_vel, 0.0001);
    }

    const Path& get_control_points() {
        return m_control_points;
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

private:
    Path m_control_points;
    Params m_params;

    Path m_path;
    Path m_left;
    Path m_right;

    Path m_center_vel;
    Path m_left_vel;
    Path m_right_vel;

    // Three split passes are issued to a trajectory. This routine calculates
    // split ratio for each pass based on initial segment count, max driving
    // time and a time step
    static std::array<int, 3> split_ratio(Double seg_count, Double max_time,
        Double step)
    {
        //return {3, 1, 1};
        int first = 0, second = 0, third = 0;
        int old_count = 0;
        int max_count = max_time / step;
        if (max_count < 100) {
            int p_first = 0, p_total = 0;
            for (int i = 4; i <= 6; i++) {
                for (int j = 1; j <= 8; j++) {
                    p_first = i * (seg_count - 1) + seg_count;
                    p_total = j * (p_first - 1) + p_first;

                    if (p_total <= max_count && p_total > old_count) {
                        first = i;
                        second = j;
                        old_count = p_total;
                    }
                }
            } 
        }
        else {
            int p_first = 0, p_second = 0, p_total = 0;
            for(int i = 1; i < 5; i ++) {
                for (int j = 1; j <= 8; j++) {
                    for (int k = 1; k < 8; k++) {
                        p_first = i * (seg_count - 1) + seg_count;
                        p_second = j * (p_first - 1) + p_first;
                        p_total = k * (p_second - 1) + p_second;
                        if (p_total < max_count) {
                            first = i;
                            second = j;
                            third = k;
                        }
                    }
                }
            }
        }

        return { first, second, third };
    }

    // Creates new path by inserting num segments inside every segmnet of
    // original path
    static Path inject(const Path& p, size_t num) {
        Path res;
        res.reserve(p.size() + num * (p.size() - 1));

        for (size_t i = 0; i < p.size() - 1; i++) {
            res.push_back(p[i]);

            for (size_t j = 1; j < num + 1; j++) {
                res.push_back(j * (p[i + 1] - p[i]) / (num + 1) + p[i]);
            }
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

    void left_right(Double robot_width) {
        std::vector<Double> gradient;
        for (size_t i = 0; i < m_path.size() - 1; i ++) {
            auto diff = m_path[i + 1] - m_path[i];
            gradient.push_back(atan2(diff.y, diff.x));
        }
        gradient.push_back(gradient.back());

        for (size_t i = 0; i < gradient.size(); i++) {
            m_left.emplace_back(
                robot_width / 2.0 * cos(gradient[i] + M_PI / 2.0) + m_path[i].x,
                robot_width / 2.0 * sin(gradient[i] + M_PI / 2.0) + m_path[i].y);
            m_right.emplace_back(
                robot_width / 2.0 * cos(gradient[i] - M_PI / 2.0) + m_path[i].x,
                robot_width / 2.0 * sin(gradient[i] - M_PI / 2.0) + m_path[i].y);
        }
    }

    // Constructs a velocity from positions
    // Result contains two values - x is a time step, y is a speed
    Path velocity(const Path& p, double time_step) {
        Path res;
        res.emplace_back(0, 0);
        for(size_t i = 1; i < p.size(); i++) {
            res.emplace_back(res.back().x + time_step,
                             ((p[i] - p[i - 1]) / time_step).length());
        }
        return res;
    }

    static std::vector<Double> error_sum(const Path& orig, const Path& smooth) {
        Double time_step = orig[1].x - orig[0].x;
        std::vector<Double> res;
        Double orig_d = orig[0].y;
        Double smooth_d = smooth[0].y;

        for (size_t i = 1; i < orig.size(); i++) {
            orig_d = orig[i].y * time_step + orig_d;
            smooth_d = smooth[i].y * time_step + smooth_d;

            res.push_back(smooth_d - orig_d);
        }

        return res;
    }

    static Path velocity_fix(Path smooth, const Path& orig, Double tolerance) {
        auto difference = error_sum(orig, smooth);

        Double increase = 0;
        while (fabs(difference.back()) > tolerance) {
            increase = difference.back() / 50;
            for (size_t i = 1; i < smooth.size() - 1; i++) 
                smooth[i].y -= increase;

            difference = error_sum(orig, smooth);
        }

        return smooth;
    }
};

using PathPlanner = PathPlannerBase<double>;
using PathPlannerSingle = PathPlannerBase<double>;

} // namespace yaqwsx