#include <memory>

#include "Gamma/Oscillator.h"
#include "al/app/al_App.hpp"
#include "al/scene/al_PolySynth.hpp"  // For asciiToMidi()
#include "al_ext/soundfile/al_OutputRecorder.hpp"
#include "al_ext/soundfile/al_SoundfileRecordGUI.hpp"

using namespace al;

struct MyApp : public App {
  OutputRecorder soundFileRecorder;
  gam::Square<> wave{440.0f};

  void onInit() override {
    // Set Gamma sampling rate. Needed for the gam::Square oscillator
    gam::sampleRate(audioIO().framesPerSecond());

    // Set writing time to 2 seconds. This will end recording automatically
    // You can also end recording programmatically by using soundfile.close()
    // if setMaxWriteTime() is not used to set max time, record will continue
    // until disk space runs out.
    soundFileRecorder.setMaxWriteTime(20.0f);
    audioIO().append(soundFileRecorder);
  }

  void onCreate() override { imguiInit(); }

  void onAnimate(double dt) {
    // Prepare GUI
    imguiBeginFrame();

    ParameterGUI::beginPanel("Record GUI");
    SoundFileRecordGUI::drawRecorderWidget(&soundFileRecorder,
                                           audioIO().framesPerSecond(),
                                           audioIO().channelsOut());
    ParameterGUI::endPanel();
    imguiEndFrame();
  }

  void onDraw(Graphics &g) {
    g.clear();

    // Draw the GUI
    imguiDraw();
  }

  void onSound(AudioIOData &io) override {
    while (io()) {
      // Write the signal to the output
      float waveValue = wave();
      io.out(0) = waveValue * 0.5;
      io.out(1) = -waveValue * 0.5;  // Phase inverted
    }
    // write the output buffers to the soundfile
    //        soundFile.write({io.outBuffer(0), io.outBuffer(1)},
    //        io.framesPerBuffer());
  }

  bool onKeyDown(Keyboard const &k) override {
    int midiVal = asciiToMIDI(k.key());
    if (midiVal > 0) {  // Change frequency from keyboard
      wave.freq(220 * pow(2, (midiVal - 69) / 12.0));
    }
    return true;
  }

  void onExit() { imguiShutdown(); }
};

int main(int argc, char *argv[]) {
  MyApp app;
  app.start();
  return 0;
}
