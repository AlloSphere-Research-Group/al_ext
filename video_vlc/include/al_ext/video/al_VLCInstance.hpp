#ifndef INCLUDE_AL_VLC_INSTANCE_HPP
#define INCLUDE_AL_VLC_INSTANCE_HPP

#include <vlc/vlc.h>
#include <memory>
#include <mutex>

namespace al {

class VLCInstance {
public:
    static VLCInstance& get() {
        static VLCInstance instance;
        return instance;
    }

    // Delete copy and move operations
    VLCInstance(const VLCInstance&) = delete;
    VLCInstance& operator=(const VLCInstance&) = delete;
    VLCInstance(VLCInstance&&) = delete;
    VLCInstance& operator=(VLCInstance&&) = delete;

    libvlc_instance_t* instance() { return mVlcInstance; }

    ~VLCInstance() {
        if (mVlcInstance) {
            libvlc_release(mVlcInstance);
            mVlcInstance = nullptr;
        }
    }

private:
    VLCInstance() {
        putenv("VLC_PLUGIN_PATH=/Applications/VLC.app/Contents/MacOS/plugins");
        putenv("VLC_VERBOSE=0");
        mVlcInstance = libvlc_new(0, nullptr);
        if (!mVlcInstance) {
            throw std::runtime_error("Failed to create VLC instance");
        }
    }

    libvlc_instance_t* mVlcInstance = nullptr;
};

} // namespace al

#endif