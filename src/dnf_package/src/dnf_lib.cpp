#pragma once
#include <torch/torch.h>
#include "../include/dnf_1d.hpp"

void DNFinit(){ //Torch debugging
    torch::Tensor myTensor = torch::rand({2,3});
    std::cout << "Here is the example tensor: " << myTensor << std::endl; //debug, cout only works when running the node only

    // Create DNF
    //DNF_1D my_dnf(10, true); //Create DNF with 10 dimensions
    //torch::Tensor input = torch::rand({10});

    // Step DNF
    //my_dnf.step(input, 0.1);

    // Get output
    //torch::Tensor output = my_dnf.get_output();
}