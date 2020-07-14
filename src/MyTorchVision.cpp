#include"MyTorchVision.h"

Mytorchvision::Mytorchvision(){
			module = torch::jit::load("../cpp_model");
}

/**
 * @brief: 数字识别
 * @param image 输入
 */
int Mytorchvision::GetNumber(const cv::Mat & image){
		    img_tensor = torch::from_blob(image.data, {1, 28, 28, 1},torch::kByte);
			img_tensor = img_tensor.permute({0, 3, 1, 2});
    		img_tensor = img_tensor.toType(torch::kFloat);
			img_tensor = img_tensor.div(255);
 			output = module.forward({img_tensor}).toTensor();
			auto max_result = output.max(1, true);
    		max_index = std::get<1>(max_result).item<float>();
			return max_index;
}