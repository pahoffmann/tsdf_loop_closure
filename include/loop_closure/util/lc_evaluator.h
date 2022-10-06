#pragma once

#include <loop_closure/path/path.h>
#include <loop_closure/util/point.h>

class Evaluator
{
private:
    // other members
    int fs_counter;
    int pose_counter;
    
    // evaluation members
    float mean_fitness_score = 0.0f;
    int num_loops = 0;
    float mean_loop_fitness_score = 0.0f;
    float min_fitness_score = 0.0f;
    float max_fitness_score = 0.0f;
    float max_loop_fitness_score = 0.0f;
    float average_prereg_fitness_score = 0.0f;

    // path and ground truth
    std::shared_ptr<Path> path_ptr_;
    std::shared_ptr<Path> ground_truth_ptr_;

public:
    Evaluator(std::shared_ptr<Path> path, std::shared_ptr<Path> ground_truth);

    ~Evaluator() = default;

    /**
     * @brief prints evaluation information
     * 
     */
    void print_evaluation();

    /**
     * @brief calculates the mean squared error between the path and the respective ground truth
     * 
     * @param path 
     * @param ground_truth 
     */
    void evaluate_against_ground_truth();

    /**
     * @brief saves the evaluation data to a csv
     * 
     */
    void save_csv();

    void update_mean_fitness_score(float fitness_score)
    {
        mean_fitness_score += fitness_score;
    }

    void increase_loop_count()
    {
        num_loops++;
    }

    void update_mean_loop_fitness_score(float fitness_score)
    {
        mean_fitness_score += fitness_score;
    }

    void update_min_fitness_score(float fitness_score)
    {
        min_fitness_score += fitness_score;
    }

    void update_max_fitness_score(float fitness_score)
    {
        max_fitness_score += fitness_score;
    }

    void update_max_loop_fitness_score(float fitness_score)
    {
        max_loop_fitness_score += fitness_score;
    }

    void update_average_prereg_fitness_score(float fitness_score)
    {
        average_prereg_fitness_score += fitness_score;
    }
};