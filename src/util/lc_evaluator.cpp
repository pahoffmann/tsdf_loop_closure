#include <loop_closure/util/lc_evaluator.h>

Evaluator::Evaluator(std::shared_ptr<Path> path, std::shared_ptr<Path> ground_truth)
{
    path_ptr_ = path;
    ground_truth_ptr_ = ground_truth;
}

void Evaluator::print_evaluation()
{

    std::cout << "####################################################################################" << std::endl;
    std::cout << "# Evaluation information: " << std::endl;
    std::cout << "# Number of Loops found: " << num_loops << std::endl;
    std::cout << "# Average preregistration fitness score: " << average_prereg_fitness_score / path_ptr_->get_length() << std::endl;
    std::cout << "# Average fitness score: " << mean_fitness_score / fs_counter << std::endl;
    std::cout << "# Average loop fitness score: " << mean_loop_fitness_score / num_loops << std::endl;
    std::cout << "# Max fitness score: " << max_fitness_score << std::endl;
    std::cout << "# Min fitness score: " << min_fitness_score << std::endl;
    std::cout << "# Max loop fitness score: " << max_loop_fitness_score << std::endl;
    std::cout << "####################################################################################" << std::endl;
}

void Evaluator::evaluate_against_ground_truth()
{
}
