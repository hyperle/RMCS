移除某功能：
class NullMotionPredictor : public interfaces::MotionPredictor {
public:
    void update(const ArmorPlate&, uint64_t) override {}
    Eigen::Vector3f predictPosition(float) override { return Eigen::Vector3f::Zero(); }
    void reset() override {}
};

// 使用时
auto null_predictor = std::make_shared<NullMotionPredictor>();
pipeline_->setMotionPredictor(null_predictor);

覆写某功能：
class NeuralNetworkLightBarDetector : public interfaces::LightBarDetector {
public:
    std::vector<LightBar> detect(const cv::Mat& frame) override {
        // 实现基于神经网络的检测算法
        // ...
        return light_bars;
    }
};

// 使用时
auto nn_detector = std::make_shared<NeuralNetworkLightBarDetector>();
pipeline_->setLightBarDetector(nn_detector);