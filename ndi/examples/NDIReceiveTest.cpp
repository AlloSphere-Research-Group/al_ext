#include "al/app/al_App.hpp"
#include "al_ext/ndi/al_NDIReceiver.hpp"

using namespace al;

class NDIReceiverApp : public App {
public:
    void onCreate() override {
        // Initialize NDI
        if (!ndiReceiver.init()) {
            std::cerr << "Failed to initialize NDI" << std::endl;
            quit();
            return;
        }

        // List available sources
        auto sources = ndiReceiver.getAvailableSources();
        for (const auto& source : sources) {
            std::cout << "Found source: " << source.name << std::endl;
        }

        // Connect to first available source
        if (!ndiReceiver.connect()) {
            std::cerr << "Failed to connect to NDI source" << std::endl;
            quit();
            return;
        }

        // Initialize texture
        tex.filter(Texture::LINEAR);
        tex.wrap(Texture::CLAMP_TO_EDGE);
    }

    void onDraw(Graphics& g) override {
        // Update texture with latest NDI frame
        if (ndiReceiver.update(tex)) {
            // Draw texture
            g.pushMatrix();
            g.translate(0, 0, -4);
            g.scale(1,-1,1);
            g.quad(tex, -1, -1, 2, 2);
            g.popMatrix();
        }
    }

private:
    NDIReceiver ndiReceiver;
    Texture tex;
};

int main() {
    NDIReceiverApp app;
    app.start();
    return 0;
}