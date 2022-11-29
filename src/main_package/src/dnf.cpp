#pragma once
#include <torch/torch.h>

void DNFinit(){
    torch::Tensor myTensor = torch::rand({2,3});
    std::cout << "Here is the example tensor: " << myTensor << std::endl; //debug, cout only works when running the node only
}