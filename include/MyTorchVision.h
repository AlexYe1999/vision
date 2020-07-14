#ifndef MY_TORCH_VISION_H
#define MY_TORCH_VISION_H
#include<torch/script.h>
#include<opencv2/opencv.hpp>
#include<memory>

class MyTorchVision{
private:
	torch::jit::script::Module module;
 	torch::Tensor imgTensor;
	torch::Tensor output;
	float max_index;
public:
	Mytorchvision();
	int GetNumber(const cv::Mat & image);
};

#endif