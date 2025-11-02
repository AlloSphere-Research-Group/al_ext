#ifndef INCLUDE_AL_NDI_RECEIVER_HPP
#define INCLUDE_AL_NDI_RECEIVER_HPP

#include <stddef.h>
#include <Processing.NDI.Lib.h>
#include <Processing.NDI.Find.h>

#include "al/graphics/al_Texture.hpp"

#include <vector>
#include <string>

namespace al {

struct Source {
    std::string name;
    std::string url;
};

class NDIReceiver {
public:
    NDIReceiver();
    ~NDIReceiver();

    bool init();
    std::vector<Source> getAvailableSources();
    bool connect(const char* sourceName = nullptr);
    
    // Update method now just works with texture
    bool update(Texture& tex);
    
    int width() const { return mWidth; }
    int height() const { return mHeight; }

private:
    NDIlib_recv_instance_t mReceiver;
    bool mInitialized;
    int mWidth;
    int mHeight;
    
    NDIReceiver(const NDIReceiver&) = delete;
    NDIReceiver& operator=(const NDIReceiver&) = delete;
};

} //

#endif