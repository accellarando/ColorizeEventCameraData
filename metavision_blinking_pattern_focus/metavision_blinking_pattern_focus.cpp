/**********************************************************************************************************************
 * Copyright (c) Prophesee S.A. - All Rights Reserved                                                                 *
 *                                                                                                                    *
 * Subject to Prophesee Metavision Licensing Terms and Conditions ("License T&C's").                                  *
 * You may not use this file except in compliance with these License T&C's.                                           *
 * A copy of these License T&C's is located in the "licensing" folder accompanying this file.                         *
 **********************************************************************************************************************/

// Tool for camera focusing by means of a blinking pattern, using Metavision Calibration SDK.

#include <functional>
#include <regex>
#include <chrono>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/core/pipeline/pipeline.h>
#include <metavision/sdk/core/pipeline/frame_composition_stage.h>
#include <metavision/sdk/core/pipeline/frame_generation_stage.h>
#include <metavision/sdk/driver/pipeline/camera_stage.h>
#include <metavision/sdk/cv/algorithms/spatio_temporal_contrast_algorithm.h>
#include <metavision/sdk/calibration/algorithms/blinking_frame_generator_algorithm.h>
#include <metavision/sdk/calibration/utils/pattern_blinker.h>
#include <metavision/sdk/calibration/configs/dft_high_freq_scorer_algorithm_config.h>
#include <metavision/sdk/calibration/algorithms/dft_high_freq_scorer_algorithm.h>
#include <metavision/sdk/ui/utils/event_loop.h>
#include <metavision/sdk/ui/pipeline/frame_display_stage.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using EventBuffer    = std::vector<Metavision::EventCD>;
using EventBufferPtr = Metavision::SharedObjectPool<EventBuffer>::ptr_type;

using FramePool = Metavision::SharedObjectPool<cv::Mat>;
using FramePtr  = FramePool::ptr_type;
using FrameData = std::pair<Metavision::timestamp, FramePtr>;

/// @brief Stage that accumulates events and keeps pixels which were activated with both polarities during the
/// accumulating period, if enough of them are found.
///
/// Produces a binary frame representing blinking pixels (0 or 255).
///   - Input : buffer of events                        : EventBufferPtr
///   - Output: timestamped frame (Blinking Chessboard) : FrameData
class BlinkingFrameGeneratorStage : public Metavision::BaseStage {
public:
    BlinkingFrameGeneratorStage(int width, int height,
                                const Metavision::BlinkingFrameGeneratorAlgorithmConfig &blinking_config) {
        blink_detector_ = std::make_unique<Metavision::BlinkingFrameGeneratorAlgorithm>(width, height, blinking_config);

        /// [SET_CD_EVENTS_CONSUMING_CALLBACK_BEGIN]
        set_consuming_callback([this](const boost::any &data) {
            try {
                auto buffer = boost::any_cast<EventBufferPtr>(data);
                if (buffer->empty())
                    return;
                successful_cb_ = false;
                blink_detector_->process_events(buffer->cbegin(), buffer->cend());
                if (!successful_cb_)
                    produce(std::make_pair(buffer->crbegin()->t, FramePtr())); // Temporal marker
            } catch (boost::bad_any_cast &c) { MV_LOG_ERROR() << c.what(); }
        });
        /// [SET_CD_EVENTS_CONSUMING_CALLBACK_END]

        /// [SET BINARY FRAME OUTPUT CALLBACK BEGIN]
        frame_pool_ = FramePool::make_bounded();
        blink_detector_->set_output_callback([this](Metavision::timestamp ts, cv::Mat &blinking_img) {
            successful_cb_        = true;
            auto output_frame_ptr = frame_pool_.acquire();
            cv::swap(blinking_img, *output_frame_ptr);
            produce(std::make_pair(ts, output_frame_ptr));
        });
        /// [SET BINARY FRAME OUTPUT CALLBACK END]
    }

private:
    FramePool frame_pool_;
    std::unique_ptr<Metavision::BlinkingFrameGeneratorAlgorithm> blink_detector_;
    bool successful_cb_;
};

/// @brief Stage that computes and produces the Discrete Fourier Transform of an image with a score depending on the
/// proportion of high frequencies.
///
/// @note It produces a frame with the High Frequency score written on it
///   - Input : timestamped frame (Blinking Chessboard)   : FrameData
///   - Output: timestamped frame (Score, White on Black) : FrameData
class DftHighFreqScorerStage : public Metavision::BaseStage {
public:
    DftHighFreqScorerStage(int width, int height, const Metavision::DftHighFreqScorerAlgorithmConfig &dft_config,
                           unsigned int header_score_width, unsigned int header_score_height) :
        header_score_width_(header_score_width),
        header_score_height_(header_score_height),
        frame_pool_(FramePool::make_bounded()) {
        high_freq_scorer_ = std::make_unique<Metavision::DftHighFreqScorerAlgorithm>(width, height, dft_config);
        set_consuming_callback([this](const boost::any &data) {
            try {
                auto ts_blinking_frame = boost::any_cast<FrameData>(data);
                auto &input_ts         = ts_blinking_frame.first;
                auto &input_frame_ptr  = ts_blinking_frame.second;

                if (!input_frame_ptr) {
                    produce(std::make_pair(input_ts, FramePtr())); // Temporal marker
                    return;
                }

                float output_score;
                /// [CHECK_DFT_FRAME_BEGIN]
                if (high_freq_scorer_->process_frame(input_ts, *input_frame_ptr, output_score)) {
                    auto output_frame_ptr = frame_pool_.acquire();
                    output_frame_ptr->create(header_score_height_, header_score_width_, CV_8UC3);
                    output_frame_ptr->setTo(0);
                    const std::string score_str = std::to_string(100 * output_score);
                    const cv::Size str_size     = cv::getTextSize(score_str, cv::FONT_HERSHEY_SIMPLEX, 1, 1, 0);
                    cv::putText(*output_frame_ptr, score_str,
                                cv::Point((output_frame_ptr->cols - str_size.width) / 2,
                                          (output_frame_ptr->rows + str_size.height) / 2),
                                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
                    produce(std::make_pair(input_ts, output_frame_ptr));
                } else {
                    produce(std::make_pair(input_ts, FramePtr())); // Temporal marker
                }
                /// [CHECK_DFT_FRAME_END]
            } catch (boost::bad_any_cast &c) { MV_LOG_ERROR() << c.what(); }
        });
    }

private:
    std::unique_ptr<Metavision::DftHighFreqScorerAlgorithm> high_freq_scorer_;
    FramePool frame_pool_;

    const int header_score_width_;
    const int header_score_height_;
};

/// @brief Stage that produces a blinking pattern at a fixed frequency
///
///   - Input : None
///   - Output: timestamped frame (Blinking Pattern) : FrameData
class PatternBlinkerStage : public Metavision::BaseStage {
public:
    PatternBlinkerStage(const std::string &pattern_image_path, int display_height,
                        Metavision::timestamp refresh_period_us) {
        stop_rendering_loop_ = false;
        refresh_period_us_   = refresh_period_us;

        if (!fs::is_regular_file(pattern_image_path))
            throw std::runtime_error("Pattern file not found at: " + pattern_image_path);

        const cv::Mat blink_pattern = cv::imread(pattern_image_path, cv::ImreadModes::IMREAD_GRAYSCALE);
        blinker_                    = std::make_unique<Metavision::PatternBlinker>(blink_pattern);

        const cv::Size img_size = blinker_->get_image_size();
        const int display_width = (img_size.width * display_height) / img_size.height;
        display_size_           = cv::Size(display_width, display_height);

        set_starting_callback([this]() { rendering_thread_ = std::thread(&PatternBlinkerStage::render_loop, this); });

        set_stopping_callback([this]() {
            stop_rendering_loop_ = true;
            if (rendering_thread_.joinable())
                rendering_thread_.join();
        });
    }

    cv::Size get_display_size() const {
        return display_size_;
    }

private:
    void render_loop() {
        Metavision::timestamp ts = 0;

        auto start = std::chrono::system_clock::now();

        while (!stop_rendering_loop_) {
            if (blinker_->update_blinking_image(ts, tmp_img_)) {
                auto output_frame_ptr = frame_pool_.acquire();
                cv::resize(tmp_img_, *output_frame_ptr, display_size_);
                produce(std::make_pair(ts, output_frame_ptr));
            }

            auto end        = std::chrono::system_clock::now();
            auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            start           = end;

            if (elapsed_us < refresh_period_us_)
                std::this_thread::sleep_for(std::chrono::microseconds(refresh_period_us_ - elapsed_us));

            ts += elapsed_us;
        }
    }

    std::atomic_bool stop_rendering_loop_;
    Metavision::timestamp refresh_period_us_;
    std::thread rendering_thread_;

    cv::Size display_size_;
    std::unique_ptr<Metavision::PatternBlinker> blinker_;
    FramePool frame_pool_;
    cv::Mat tmp_img_;
};

// Application's parameters
struct Config {
    // Input/Output parameters
    std::string raw_file_path_;

    // Blinking frame generator algorithm's parameters
    Metavision::timestamp accumulation_time_;
    int min_num_blinking_pixels_;
    float blinking_pixels_ratios_on_;
    float blinking_pixels_ratios_off_;

    // Discrete Fourier Transform's parameters
    Metavision::timestamp refresh_period_us_;
    bool use_inverted_gray_ = false;

    // Pattern Blinker's parameters
    std::string pattern_image_path_;
    int pattern_blinker_height_;
    Metavision::timestamp pattern_blinker_refresh_period_us_;
};

bool get_pipeline_configuration(int argc, char *argv[], Config &config) {
    const std::string program_desc(
        "Tool for camera focusing by means of a blinking pattern, using Metavision Calibration SDK.\n");

    po::options_description options_desc;
    po::options_description base_options("Base options");
    // clang-format off
    base_options.add_options()
        ("help,h", "Produce help message.")
        ("input-raw-file,i", po::value<std::string>(&config.raw_file_path_), "Path to input RAW file. If not specified, the camera live stream is used.")
        ;
    // clang-format on

    po::options_description blinking_frame_generator_options("Blinking Frame Generator options");
    // clang-format off
    blinking_frame_generator_options.add_options()
        ("accumulation-time,a", po::value<Metavision::timestamp>(&config.accumulation_time_)->default_value(2e5), "Window of time during which events are considered to detect if a pixel is blinking.")
        ("min-blink-pix,m", po::value<int>(&config.min_num_blinking_pixels_)->default_value(0), "Minimum number of pixels needed to be detected before outputting a frame.")
        ("ratio-on",        po::value<float>(&config.blinking_pixels_ratios_on_)->default_value(1.0f), "The acceptable ratio of pixels that received only positive events over the number of pixels that received both during the accumulation window.")
        ("ratio-off",       po::value<float>(&config.blinking_pixels_ratios_off_)->default_value(1.0f),  "The acceptable ratio of pixels that received only negative events over the number of pixels that received both during the accumulation window.")
        ;
    // clang-format on

    po::options_description dft_options("Discrete Fourier Transform options");
    // clang-format off
    dft_options.add_options()
        ("dft-refresh", po::value<Metavision::timestamp >(&config.refresh_period_us_)->default_value(1e4), "Time period between two consecutive process (skip the blinking frames that are too close in time to the last one processed).")
        ("invert-gray", po::bool_switch(&config.use_inverted_gray_), "Invert the gray levels so that white becomes black (and conversely).")
        ;
    // clang-format on

    po::options_description pattern_blinker_options("Pattern Blinker options");
    // clang-format off
    pattern_blinker_options.add_options()
        ("pattern-image-path",  po::value<std::string>(&config.pattern_image_path_), "If a path to a pattern file is provided, display a blinking pattern on screen")
        ("pattern-blinker-height",  po::value<int>(&config.pattern_blinker_height_)->default_value(1080), "Height of the blinking pattern.")
        ("pattern-blinker-refresh", po::value<Metavision::timestamp>(&config.pattern_blinker_refresh_period_us_)->default_value(1e4), "Refresh period of the pattern blinker in us.")
        ;
    // clang-format on

    options_desc.add(base_options).add(blinking_frame_generator_options).add(dft_options).add(pattern_blinker_options);

    po::variables_map vm;
    try {
        po::store(po::command_line_parser(argc, argv).options(options_desc).run(), vm);
        po::notify(vm);
    } catch (po::error &e) {
        MV_LOG_ERROR() << program_desc;
        MV_LOG_ERROR() << options_desc;
        MV_LOG_ERROR() << "Parsing error:" << e.what();
        return false;
    }

    if (vm.count("help")) {
        MV_LOG_INFO() << program_desc;
        MV_LOG_INFO() << options_desc;
        return false;
    }

    return true;
}

int main(int argc, char *argv[]) {
    Config conf_;

    if (!get_pipeline_configuration(argc, argv, conf_))
        return 1;

    const auto start = std::chrono::high_resolution_clock::now();

    Metavision::Pipeline p(true);

    Metavision::Camera camera;
    if (conf_.raw_file_path_.empty()) {
        try {
            camera = Metavision::Camera::from_first_available();
        } catch (const Metavision::CameraException &e) {
            MV_LOG_ERROR() << e.what();
            return 1;
        }
    } else {
        camera = Metavision::Camera::from_file(conf_.raw_file_path_, false);
    }

    const unsigned short width  = camera.geometry().width();
    const unsigned short height = camera.geometry().height();

    const unsigned int header_score_width                = width;
    const unsigned int header_score_height               = 50;
    const Metavision::timestamp event_buffer_duration_ms = 100;
    const int display_fps                                = 10;

    // Pipeline
    //
    //  0 (Cam) -->-- 1 (STC) ---------->---------- 3 (Blink Frame Gen) -->---|
    //                |                             |                         |
    //                v                             v                         5 (Frame Composer) -->-- 6 (Display)
    //                |                             |                         |
    //                2 (Events Frame Gen)          4 (DFT) -------------->---|
    //                |                                                       |
    //                +------------------>-------------------------------->---|
    //
    //
    //  (optional)                  7 (Pattern Blinker) ----->----- 8 (Display)

    // 0) Camera stage
    auto &cam_stage =
        p.add_stage(std::make_unique<Metavision::CameraStage>(std::move(camera), event_buffer_duration_ms));

    // 1) STC Stage
    constexpr Metavision::timestamp stc_ths = 1e4;
    auto &stc_stage                         = p.add_algorithm_stage(
        std::make_unique<Metavision::SpatioTemporalContrastAlgorithm>(width, height, stc_ths, true), cam_stage);

    // 2) Events frame stage
    auto &events_frame_stage = p.add_stage(
        std::make_unique<Metavision::FrameGenerationStage>(width, height, event_buffer_duration_ms, display_fps),
        stc_stage);

    // 3) Blinking Frame Generator stage
    Metavision::BlinkingFrameGeneratorAlgorithmConfig blinking_config(
        conf_.accumulation_time_, conf_.min_num_blinking_pixels_, conf_.blinking_pixels_ratios_on_,
        conf_.blinking_pixels_ratios_off_);
    auto &blinking_frame_generator_stage =
        p.add_stage(std::make_unique<BlinkingFrameGeneratorStage>(width, height, blinking_config), stc_stage);

    // 4) Discrete Fourier Transform Stage
    Metavision::DftHighFreqScorerAlgorithmConfig dft_config(conf_.refresh_period_us_, conf_.use_inverted_gray_);
    auto &high_freq_score_stage = p.add_stage(
        std::make_unique<DftHighFreqScorerStage>(width, height, dft_config, header_score_width, header_score_height),
        blinking_frame_generator_stage);

    // 5) Frame composer stage
    /// [CONNECT_FRAME_COMPOSER_BEGIN]
    auto &frame_composer_stage = p.add_stage(std::make_unique<Metavision::FrameCompositionStage>(display_fps, 0));
    frame_composer_stage.add_previous_frame_stage(high_freq_score_stage, width + 10, 0, header_score_width,
                                                  header_score_height);
    frame_composer_stage.add_previous_frame_stage(events_frame_stage, 0, header_score_height + 10, width, height);
    frame_composer_stage.add_previous_frame_stage(blinking_frame_generator_stage, width + 10, header_score_height + 10,
                                                  width, height);
    /// [CONNECT_FRAME_COMPOSER_END]
    // 6) Stage displaying the raw events and the blinking events, with a header band showing the high frequency score
    const auto composed_width  = frame_composer_stage.frame_composer().get_total_width();
    const auto composed_height = frame_composer_stage.frame_composer().get_total_height();
    auto &display_stage =
        p.add_stage(std::make_unique<Metavision::FrameDisplayStage>(
                        "Raw events, Blinking events and High Frequency score ", composed_width, composed_height),
                    frame_composer_stage);

    if (!conf_.pattern_image_path_.empty()) {
        try {
            // 7) Pattern Blinker Stage
            auto &pattern_blinker_stage = p.add_stage(std::make_unique<PatternBlinkerStage>(
                conf_.pattern_image_path_, conf_.pattern_blinker_height_, conf_.pattern_blinker_refresh_period_us_));

            // 8) Stage displaying the blinking pattern
            const auto &blinking_pattern_size = pattern_blinker_stage.get_display_size();
            auto &disp_blinker_stage =
                p.add_stage(std::make_unique<Metavision::FrameDisplayStage>(
                                "Blinking Pattern", blinking_pattern_size.width, blinking_pattern_size.height,
                                Metavision::Window::RenderMode::GRAY),
                            pattern_blinker_stage);
        } catch (const std::runtime_error &e) {
            MV_LOG_ERROR() << e.what();
            return 1;
        }
    }

    // Run the pipeline and wait for its completion
    p.run();

    const auto end     = std::chrono::high_resolution_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    MV_LOG_INFO() << "Ran in" << static_cast<float>(elapsed.count()) / 1000.f << "s";

    return 0;
}
