
#include "al/app/al_App.hpp"
#include "al/graphics/al_Graphics.hpp"
#include "al/graphics/al_Mesh.hpp"
#include "al_ext/video/al_VideoPlayer.hpp"

using namespace al;

class VideoApp : public App {
public:
    void onCreate() override {
        // Load video file

        if (!mVideoPlayer.open("/Users/fishuyo/Downloads/aforest_0213.mp4") ) {
            std::cerr << "Failed to load video" << std::endl;
            quit();
            return;
        }
        
        // Start playback
        mVideoPlayer.play();
        
        // Create mesh for video display
        mVideoMesh.primitive(Mesh::TRIANGLE_STRIP);
        mVideoMesh.vertex(-1, -1, 0);
        mVideoMesh.vertex( 1, -1, 0);
        mVideoMesh.vertex(-1,  1, 0);
        mVideoMesh.vertex( 1,  1, 0);
        
        mVideoMesh.texCoord(0, 1);
        mVideoMesh.texCoord(1, 1);
        mVideoMesh.texCoord(0, 0);
        mVideoMesh.texCoord(1, 0);
    }
    
    void onDraw(Graphics& g) override {
        g.clear(0);
        
        // Draw video texture
        g.pushMatrix();
        g.translate(0, 0, -4);
        mVideoPlayer.update();
        mVideoPlayer.texture().bind();
        g.texture();
        g.draw(mVideoMesh);
        g.popMatrix();
    }
    
    bool onKeyDown(const Keyboard& k) override {
        if (k.key() == ' ') {
            if (mVideoPlayer.isPlaying()) {
                mVideoPlayer.pause();
            } else {
                mVideoPlayer.play();
            }
        }
        return true;
    }
    
private:
    VideoPlayer mVideoPlayer;
    Mesh mVideoMesh;
};

int main() { VideoApp().start(); }