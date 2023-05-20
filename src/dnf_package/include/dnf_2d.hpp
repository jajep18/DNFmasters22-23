#ifndef DNF_2D_HPP
#define DNF_2D_HPP
/* 
 * Description:     Implementation of 2-dimensional dynamic neural field.
 *                  Uses Torch for data storage and processing (step)
 *
 * Author:			Erik Lindby
 *                  Jacob Floe Jeppesen
 *					University of Southern Denmark
 * Creation date:   01-11-2022
 */

// Includes
#include <torch/torch.h>


class DNF_2D 
{
public:
    // Constructors / Destructors
    DNF_2D(); // Explicit default constructor
    DNF_2D(int _dimensions1, int _dimensions2, bool debug = true, int _learningrule = 1, int _normalization = 1, int _suppression = 1, float _dt = 0.1);
    ~DNF_2D();

    // Process step
    void step(torch::Tensor input1, torch::Tensor input2);

    // Getters
    torch::Tensor get_activation();
    torch::Tensor get_output();
    torch::Tensor get_input(int index_input);
    torch::Tensor extract_response_DNF(torch::Tensor input, int index_input = 0);
    float get_activation_at(int index1, int index2);

    // Setters
    void set_activation(torch::Tensor activation);
    void set_output(torch::Tensor output);
    void set_input(torch::Tensor input1, torch::Tensor input2);
    void set_input_element(int index_input, int index_element, float value);
    void reset_input();
    
private:
    int m_dimensions1;       //Dimension in DNF (Activation, Output, Input)
    int m_dimensions2;       //Dimension in DNF (Activation, Output, Input)
    int m_minimum_dims = 3; //Minimum dimensions in DNF for interaction kernel to work

    // Activation
    torch::Tensor m_activation;

    // Output
    torch::Tensor m_output; //Multidimensional tensor

    // Input
    torch::Tensor m_input1;
    torch::Tensor m_input2;

    // Interaction kernel
    torch::Tensor m_interaction_kernel;
    /* TODO: Set this as its own class thats passed to make
     * machine learning of architectures easier.
     * INSTEAD: We pass the parameters for learning in the constructor to the variables below
     */

    int learningrule;
    int normalization;
    int suppression;
    float dt;
};

#endif 