#ifndef DNF_1D_HPP
#define DNF_1D_HPP
/* 
 * Description:     Implementation of 1-dimensional dynamic neural field.
 *                  Uses Torch for data storage and processing (step)
 *
 * Author:			Erik Lindby
 *                  Jacob Floe Jeppesen
 *					University of Southern Denmark
 * Creation date:   01-11-2022
 */

// Includes
#include <torch/torch.h>


class DNF_1D 
{
public:
    // Constructors / Destructors
    DNF_1D(); // Explicit default constructor
    DNF_1D(int _dimensions, bool debug = true);
    ~DNF_1D();

    // Process step
    void step(torch::Tensor input, float dt);

    // Getters
    torch::Tensor get_activation();
    torch::Tensor get_output();
    torch::Tensor get_input();

    // Setters
    void set_activation(torch::Tensor activation);
    void set_output(torch::Tensor output);
    void set_input(torch::Tensor input);
    void set_input_element(int index, float value);
    void reset_input();
    
private:
    int m_dimensions;       //Dimension in DNF (Activation, Output, Input)
    int m_minimum_dims = 3; //Minimum dimensions in DNF for interaction kernel to work

    // Activation
    torch::Tensor m_activation;

    // Output
    torch::Tensor m_output;

    // Input
    torch::Tensor m_input;

    // Interaction kernel
    torch::Tensor m_interaction_kernel;
    /* TODO: Set this as its own class thats passed to make
     * machine learning of architectures easier.
     */
};

#endif 