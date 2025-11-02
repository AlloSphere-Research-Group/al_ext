#include "al_ext/ndi/al_NDIReceiver.hpp"
#include <iostream>

namespace al {



std::vector<Source> NDIReceiver::getAvailableSources() {
    std::vector<Source> sources;
    if (!mInitialized) return sources;

    NDIlib_find_instance_t finder = NDIlib_find_create_v2();
    if (!finder) return sources;

    uint32_t num_sources = 0;
    const NDIlib_source_t* ndi_sources = nullptr;
    
    // Wait up to 1 second for sources
    NDIlib_find_wait_for_sources(finder, 1000);
    ndi_sources = NDIlib_find_get_current_sources(finder, &num_sources);

    for (uint32_t i = 0; i < num_sources; i++) {
        Source source;
        source.name = ndi_sources[i].p_ndi_name;
        source.url = ndi_sources[i].p_url_address;
        sources.push_back(source);
    }

    NDIlib_find_destroy(finder);
    return sources;
}

NDIReceiver::NDIReceiver()
    : mReceiver(nullptr)
    , mInitialized(false)
    , mWidth(0)
    , mHeight(0)
{}

NDIReceiver::~NDIReceiver() {
    if (mReceiver) {
        NDIlib_recv_destroy(mReceiver);
        mReceiver = nullptr;
    }
    if (mInitialized) {
        NDIlib_destroy();
    }
}

bool NDIReceiver::init() {
    if (!NDIlib_initialize()) {
        std::cerr << "Failed to initialize NDI" << std::endl;
        return false;
    }
    mInitialized = true;
    return true;
}

bool NDIReceiver::connect(const char* sourceName) {
    if (!mInitialized) {
        std::cerr << "NDI not initialized" << std::endl;
        return false;
    }

    // Create a finder to locate NDI sources
    NDIlib_find_instance_t finder = NDIlib_find_create_v2();
    if (!finder) {
        std::cerr << "Failed to create NDI finder" << std::endl;
        return false;
    }

    // Wait for sources to be available
    uint32_t num_sources = 0;
    const NDIlib_source_t* sources = nullptr;
    
    // Wait up to 5 seconds for sources
    for (int i = 0; i < 50 && num_sources == 0; i++) {
        NDIlib_find_wait_for_sources(finder, 100); // Wait 100ms
        sources = NDIlib_find_get_current_sources(finder, &num_sources);
    }

    if (num_sources == 0) {
        std::cerr << "No NDI sources found" << std::endl;
        NDIlib_find_destroy(finder);
        return false;
    }

    // Select the source
    const NDIlib_source_t* selected_source = nullptr;
    if (sourceName) {
        // Look for specific source
        for (uint32_t i = 0; i < num_sources; i++) {
            if (strcmp(sources[i].p_ndi_name, sourceName) == 0) {
                selected_source = &sources[i];
                break;
            }
        }
        if (!selected_source) {
            std::cerr << "Specified source '" << sourceName << "' not found" << std::endl;
            NDIlib_find_destroy(finder);
            return false;
        }
    } else {
        // Use first available source
        selected_source = &sources[0];
        std::cout << "Connected to first available source: " << selected_source->p_ndi_name << std::endl;
    }

    // Create the receiver
    NDIlib_recv_create_v3_t receiverDesc;
    receiverDesc.source_to_connect_to = *selected_source;
    receiverDesc.color_format = NDIlib_recv_color_format_BGRX_BGRA;
    receiverDesc.bandwidth = NDIlib_recv_bandwidth_highest;
    receiverDesc.allow_video_fields = false;

    mReceiver = NDIlib_recv_create_v3(&receiverDesc);
    
    // Clean up finder
    NDIlib_find_destroy(finder);

    if (!mReceiver) {
        std::cerr << "Failed to create NDI receiver" << std::endl;
        return false;
    }

    return true;
}

bool NDIReceiver::update(Texture& tex) {
    if (!mReceiver) return false;

    NDIlib_video_frame_v2_t videoFrame;
    NDIlib_frame_type_e frameType = NDIlib_recv_capture_v2(
        mReceiver, &videoFrame, nullptr, nullptr, 1000
    );

    if (frameType == NDIlib_frame_type_video) {
        // If texture dimensions changed, update the texture
        if (mWidth != videoFrame.xres || mHeight != videoFrame.yres) {
            mWidth = videoFrame.xres;
            mHeight = videoFrame.yres;
            
            // Configure texture format and resize
            // tex.format(GL_RGBA);
            // tex.type(GL_UNSIGNED_BYTE);
            tex.resize(mWidth, mHeight);
        }

        // Update texture with new frame data
        tex.submit(videoFrame.p_data, GL_BGRA, GL_UNSIGNED_BYTE);

        // Free the video frame
        NDIlib_recv_free_video_v2(mReceiver, &videoFrame);
        return true;
    }

    return false;
}

} // namespace al