class Evaluator
{
private:
    // evaluation members
    float mean_fitness_score = 0.0f;
    int num_loops = 0;
    float mean_loop_fitness_score = 0.0f;
    float min_fitness_score = 0.0f;
    float max_fitness_score = 0.0f;
    float max_loop_fitness_score = 0.0f;
    float average_prereg_fitness_score = 0.0f;

public:
    Evaluator(/* args */);
    ~Evaluator();
};

Evaluator::Evaluator(/* args */)
{
}

Evaluator::~Evaluator()
{
}
