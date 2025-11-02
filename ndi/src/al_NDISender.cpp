#include "al_ext/ndi/al_NDISender.hpp"
#include <iostream>

namespace al {

NDISender::NDISender()
    : mSender(nullptr)
    , mInitialized(false)
    , mHardwareEnabled(false)
{
    memset(&mHardwareCtx, 0, sizeof(mHardwareCtx));
}

NDISender::~NDISender() {
    cleanupHardwareContext();
    if (mSender) {
        NDIlib_send_destroy(mSender);
        mSender = nullptr;
    }
}

bool NDISender::init(const char* senderName, const VideoConfig& config, bool enableHardware) {
    if (!NDIlib_initialize()) {
        std::cerr << "Failed to initialize NDI" << std::endl;
        return false;
    }

    NDIlib_send_create_t desc;
    desc.p_ndi_name = senderName;
    desc.p_groups = nullptr;
    desc.clock_video = true;
    desc.clock_audio = false;

    mSender = NDIlib_send_create(&desc);
    if (!mSender) {
        std::cerr << "Failed to create NDI sender" << std::endl;
        return false;
    }

    mInitialized = true;

    if (enableHardware) {
        mHardwareEnabled = initHardwareContext(config.width, config.height);
        if (!mHardwareEnabled) {
            std::cout << "Hardware acceleration not available, falling back to software mode" << std::endl;
        }
    }

    return true;
}

bool NDISender::initHardwareContext(int width, int height) {
    // Create persistent shared texture
    glGenTextures(1, &mHardwareCtx.sharedTexture);
    glBindTexture(GL_TEXTURE_2D, mHardwareCtx.sharedTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Create persistent FBO for copying
    glGenFramebuffers(1, &mHardwareCtx.copyFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, mHardwareCtx.copyFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 
                          GL_TEXTURE_2D, mHardwareCtx.sharedTexture, 0);

    // Check FBO status
    GLenum status = glCheckFramebufferStatus(GL_FRAMEBUFFER);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (status != GL_FRAMEBUFFER_COMPLETE) {
        cleanupHardwareContext();
        std::cerr << "Failed to create hardware context FBO" << std::endl;
        return false;
    }

    // Initialize video frame structure
    mHardwareCtx.videoFrame.FourCC = NDIlib_FourCC_type_BGRA;
    mHardwareCtx.videoFrame.frame_format_type = NDIlib_frame_format_type_progressive;
    mHardwareCtx.videoFrame.timecode = NDIlib_send_timecode_synthesize;
    mHardwareCtx.videoFrame.p_data = (uint8_t*)mHardwareCtx.sharedTexture;
    mHardwareCtx.videoFrame.xres = width;
    mHardwareCtx.videoFrame.yres = height;
    mHardwareCtx.videoFrame.picture_aspect_ratio = (float)width / (float)height;
    mHardwareCtx.videoFrame.frame_rate_N = 60000;
    mHardwareCtx.videoFrame.frame_rate_D = 1000;

    mHardwareCtx.width = width;
    mHardwareCtx.height = height;
    mHardwareCtx.needsResize = false;

    return true;
}

void NDISender::cleanupHardwareContext() {
    if (mHardwareCtx.copyFBO) {
        glDeleteFramebuffers(1, &mHardwareCtx.copyFBO);
        mHardwareCtx.copyFBO = 0;
    }
    if (mHardwareCtx.sharedTexture) {
        glDeleteTextures(1, &mHardwareCtx.sharedTexture);
        mHardwareCtx.sharedTexture = 0;
    }
}

bool NDISender::resizeHardwareContext(int width, int height) {
    if (width == mHardwareCtx.width && height == mHardwareCtx.height) {
        return true;
    }

    // Resize shared texture
    glBindTexture(GL_TEXTURE_2D, mHardwareCtx.sharedTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Update video frame info
    mHardwareCtx.videoFrame.xres = width;
    mHardwareCtx.videoFrame.yres = height;
    mHardwareCtx.videoFrame.picture_aspect_ratio = (float)width / (float)height;

    mHardwareCtx.width = width;
    mHardwareCtx.height = height;
    mHardwareCtx.needsResize = false;

    return true;
}

bool NDISender::resize(int width, int height) {
    if (!mHardwareEnabled) return false;
    mHardwareCtx.needsResize = true;
    return resizeHardwareContext(width, height);
}

bool NDISender::sendDirect(GLuint textureId) {
    if (!mInitialized || !mHardwareEnabled) return false;

    // Get the dimensions of the input texture
    GLint width, height;
    glBindTexture(GL_TEXTURE_2D, textureId);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
    glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
    glBindTexture(GL_TEXTURE_2D, 0);

    // Resize if needed
    if (width != mHardwareCtx.width || height != mHardwareCtx.height) {
        if (!resizeHardwareContext(width, height)) {
            return false;
        }
    }

    // Save current FBO binding
    GLint previousFBO;
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previousFBO);

    // Copy the texture using the persistent FBO
    glBindFramebuffer(GL_FRAMEBUFFER, mHardwareCtx.copyFBO);
    
    // Bind source texture
    glBindTexture(GL_TEXTURE_2D, textureId);
    
    // Copy using a blit operation for potentially better performance
    glBlitFramebuffer(0, 0, width, height,
                     0, 0, width, height,
                     GL_COLOR_BUFFER_BIT, GL_NEAREST);

    // Restore previous state
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, previousFBO);

    // Send the frame
    NDIlib_send_send_video_v2(mSender, &mHardwareCtx.videoFrame);

    return true;
}

// bool NDISender::sendDirect(FBO& fbo) {
//     return sendDirect(fbo.tex());
// }

bool NDISender::sendDirect(Texture& tex) {
    return sendDirect(tex.id());
}

} // namespace al