#ifndef INCLUDE_AL_NDI_SENDER_HPP
#define INCLUDE_AL_NDI_SENDER_HPP

#include <stddef.h>
#include <Processing.NDI.Lib.h>
#include "al/graphics/al_FBO.hpp"
#include "al/graphics/al_Texture.hpp"

namespace al {

class NDISender {
public:
    struct VideoConfig {
        VideoConfig() : width(1920), height(1080), frameRateN(60000), frameRateD(1000) {}
        int width;
        int height;
        int frameRateN;
        int frameRateD;
    };

    NDISender();
    ~NDISender();

    // Initialize with video configuration
    bool init(const char* senderName, const VideoConfig& config = VideoConfig(), 
             bool enableHardware = true);

    // Direct GPU methods
    bool sendDirect(GLuint textureId);
    bool sendDirect(FBO& fbo);
    bool sendDirect(Texture& tex);

    // Resize the sender if input dimensions change
    bool resize(int width, int height);

    bool isInitialized() const { return mInitialized; }
    bool isHardwareEnabled() const { return mHardwareEnabled; }

private:
    NDIlib_send_instance_t mSender;
    bool mInitialized;
    bool mHardwareEnabled;
    
    struct HardwareContext {
        GLuint sharedTexture;     // Persistent shared texture
        GLuint copyFBO;           // Persistent FBO for copying
        int width;
        int height;
        NDIlib_video_frame_v2_t videoFrame;
        bool needsResize;
    } mHardwareCtx;
    
    bool initHardwareContext(int width, int height);
    void cleanupHardwareContext();
    bool resizeHardwareContext(int width, int height);
    
    NDISender(const NDISender&) = delete;
    NDISender& operator=(const NDISender&) = delete;
};

} // namespace al

#endif