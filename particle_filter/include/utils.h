#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <random>
#include <geometry_msgs/Quaternion.h>

namespace Utils {
    double randomNormal(double mean, double stddev) {
        static std::default_random_engine generator;
        std::normal_distribution<double> distribution(mean, stddev);
        return distribution(generator);
    }

    double quaternionToAngle(const geometry_msgs::Quaternion& q) {
        double roll, pitch, yaw;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(q, quat);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        return yaw;
    }

    std::vector<std::vector<double>> rotationMatrix(double angle) {
        return {{cos(angle), -sin(angle)}, {sin(angle), cos(angle)}};
    }
    std::vector<int> randomChoice(int max_particles, int num_samples, const std::vector<double>& weights) {
        std::vector<int> sampled_indices(num_samples);
        std::discrete_distribution<> dist(weights.begin(), weights.end());
        std::default_random_engine generator;

        for (int i = 0; i < num_samples; ++i) {
            sampled_indices[i] = dist(generator) % max_particles;
        }

        return sampled_indices;
    }


    std_msgs::Header makeHeader(const std::string& frame_id, const ros::Time& stamp = ros::Time::now()) {
        std_msgs::Header header;
        header.stamp = stamp;
        header.frame_id = frame_id;
        return header;
    }

    geometry_msgs::Quaternion angleToQuaternion(double angle) {
        tf::Quaternion quat = tf::createQuaternionFromYaw(angle);
        geometry_msgs::Quaternion q;
        tf::quaternionTFToMsg(quat, q);
        return q;
    }

    std::vector<geometry_msgs::Pose> particlesToPoses(const std::vector<std::vector<double>>& particles) {
        std::vector<geometry_msgs::Pose> poses(particles.size());
        for (size_t i = 0; i < particles.size(); ++i) {
            poses[i].position.x = particles[i][0];
            poses[i].position.y = particles[i][1];
            poses[i].orientation = angleToQuaternion(particles[i][2]);
        }
        return poses;
    }

    void mapToWorld(std::vector<std::vector<double>>& particles, const nav_msgs::MapMetaData& map_info) {
        double scale = map_info.resolution;
        double angle = quaternionToAngle(map_info.origin.orientation);

        // Precompute cosine and sine of the rotation angle
        double c = cos(angle);
        double s = sin(angle);

        for (auto& particle : particles) {
            // Rotation
            double temp_x = particle[0];
            particle[0] = c * particle[0] - s * particle[1];
            particle[1] = s * temp_x + c * particle[1];

            // Scale
            particle[0] *= scale;
            particle[1] *= scale;

            // Translate
            particle[0] += map_info.origin.position.x;
            particle[1] += map_info.origin.position.y;

            // Adjust the orientation angle
            particle[2] += angle;
        }
    }



    class CircularArray {
    public:
        CircularArray(size_t size) : arr(size, 0.0), ind(0), num_els(0) {}

        void append(double value) {
            if (num_els < arr.size()) {
                ++num_els;
            }
            arr[ind] = value;
            ind = (ind + 1) % arr.size();
        }

        double mean() const {
            double sum = std::accumulate(arr.begin(), arr.begin() + num_els, 0.0);
            return sum / num_els;
        }

        double median() const {
            std::vector<double> sorted_arr(arr.begin(), arr.begin() + num_els);
            std::sort(sorted_arr.begin(), sorted_arr.end());
            if (num_els % 2 == 0) {
                return (sorted_arr[num_els / 2 - 1] + sorted_arr[num_els / 2]) / 2.0;
            } else {
                return sorted_arr[num_els / 2];
            }
        }

    private:
        std::vector<double> arr;
        size_t ind;
        size_t num_els;
    };

    // Timer class to compute the rate of events
    class Timer {
    public:
        Timer(size_t smoothing) : arr(smoothing), last_time(std::chrono::high_resolution_clock::now()) {}

        void tick() {
            auto t = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = t - last_time;
            arr.append(1.0 / elapsed.count());
            last_time = t;
        }

        double fps() const {
            return arr.mean();
        }

    private:
        CircularArray arr;
        std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    };
}

#endif // UTILS_H
