// #include "al_ext/video/al_VideoTexture.hpp"
// #include "al/graphics/al_Graphics.hpp"

// namespace al {

// VideoTexture::VideoTexture() {}

// VideoTexture::~VideoTexture() {
//     if (mTextureId) {
//         glDeleteTextures(1, &mTextureId);
//     }
// }

// bool VideoTexture::init(int width, int height) {
//     std::lock_guard<std::mutex> lock(mTextureMutex);
    
//     mWidth = width;
//     mHeight = height;

//     if (mTextureId == 0) {
//         glGenTextures(1, &mTextureId);
//     }

//     glBindTexture(GL_TEXTURE_2D, mTextureId);
//     glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
//     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
//     glBindTexture(GL_TEXTURE_2D, 0);

//     return true;
// }

// void VideoTexture::update(const unsigned char* pixels) {
//     std::lock_guard<std::mutex> lock(mTextureMutex);
    
//     if (mTextureId && pixels) {
//         glBindTexture(GL_TEXTURE_2D, mTextureId);
//         glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, mWidth, mHeight, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
//         glBindTexture(GL_TEXTURE_2D, 0);
//     }
// }

// void VideoTexture::bind(Graphics& g) {
//     std::lock_guard<std::mutex> lock(mTextureMutex);
//     g.texture(mTextureId);
// }

// } // namespace al