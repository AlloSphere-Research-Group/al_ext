#ifndef AL_DISKBUFFER_HPP
#define AL_DISKBUFFER_HPP

#include <string>

#include "al/io/al_File.hpp"
#include "al/ui/al_Parameter.hpp"
#include "al/ui/al_ParameterServer.hpp"
#include "al_ext/tinc/al_BufferManager.hpp"
#include "nlohmann/json.hpp"

namespace al {

template <class DataType = nlohmann::json>
class DiskBuffer : public BufferManager<DataType> {
public:
  DiskBuffer(std::string fileName, std::string path = "", uint16_t size = 2)
      : BufferManager<DataType>(size) {
    m_fileName = fileName;
    m_path = File::conformDirectory(path);
  }

  bool updateData() {
    std::ifstream file(m_path + m_fileName);
    if (file.good()) {
      return parseFile(file, getWritable());
    } else {
      return false;
    }
  }

  virtual bool parseFile(std::ifstream &file,
                         std::shared_ptr<DataType> newData) {

    *newData = nlohmann::json::parse(file);
    BufferManager<DataType>::doneWriting(newData);

    return true;
  }

  void exposeToNetwork(ParameterServer &p) {
    if (m_trigger) {
      std::cerr << "ERROR: already registered. Aborting." << std::endl;
      return;
    }
    std::string pathPrefix = "/__DiskBuffer";
    if (m_fileName[0] != '/') {
      pathPrefix += "/";
    }
    m_trigger = std::make_shared<Trigger>(pathPrefix + m_fileName);
    p.registerParameter(*m_trigger);
    m_trigger->registerChangeCallback(
        [this](float value) { this->updateData(); });
    // There will be problems if this object is destroyed before the parameter
    // server Should this be a concern?
  }

private:
  // Make this function private as users should not have a way to make the
  // buffer writable. Data writing should be done by writing to the file.
  using BufferManager<DataType>::getWritable;

  std::string m_fileName;
  std::string m_path;
  std::shared_ptr<Trigger> m_trigger;
};
} // namespace al

#endif // AL_DISKBUFFER_HPP
