// #ifndef INCLUDE_AL_VIDEO_TEXTURE_HPP
// #define INCLUDE_AL_VIDEO_TEXTURE_HPP

// //#include "al/graphics/al_Graphics.hpp"
// #include <mutex>

// namespace al {

// class VideoTexture {
// public:
//     VideoTexture();
//     ~VideoTexture();

//     // Initialize texture with dimensions
//     bool init(int width, int height);

//     // Update texture with new pixel data
//     void update(const unsigned char* pixels);

//     // Bind texture for rendering
//     void bind(Graphics& g);

//     // Get texture dimensions
//     int width() const { return mWidth; }
//     int height() const { return mHeight; }

//     // Get OpenGL texture ID
//     unsigned int textureId() const { return mTextureId; }

// private:
//     int mWidth{0};
//     int mHeight{0};
//     unsigned int mTextureId{0};
//     std::mutex mTextureMutex;
// };

// } // namespace al

// #endif