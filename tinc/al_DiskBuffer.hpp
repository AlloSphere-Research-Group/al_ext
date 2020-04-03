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
  DiskBuffer(std::string name, std::string fileName = "", std::string path = "",
             uint16_t size = 2)
      : BufferManager<DataType>(size) {
    m_name = name;
    // TODO there should be a check through a singleton to make sure names are
    // unique
    m_fileName = fileName;
    if (path.size() > 0) {
      m_path = File::conformDirectory(path);
    } else {
      m_path = "";
    }
  }

  bool updateData(std::string filename = "") {
    if (filename.size() > 0) {
      m_fileName = filename;
    }
    std::ifstream file(m_path + m_fileName);
    if (file.good()) {
      return parseFile(file, getWritable());
    } else {
      std::cerr << "Error code: " << strerror(errno);
      return false;
    }
  }

  // Careful, this is not thread safe. Needs to be called synchronously to any
  // process functions
  std::string getCurrentFileName() { return m_fileName; }

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
    std::string pathPrefix = "/__DiskBuffer/";
    //    if (m_fileName[0] != '/') {
    //      pathPrefix += "/";
    //    }
    m_trigger = std::make_shared<ParameterString>(pathPrefix + m_name);
    p.registerParameter(*m_trigger);
    m_trigger->registerChangeCallback(
        [this](std::string value) { this->updateData(value); });
    // There will be problems if this object is destroyed before the parameter
    // server Should this be a concern?
  }

private:
  // Make this function private as users should not have a way to make the
  // buffer writable. Data writing should be done by writing to the file.
  using BufferManager<DataType>::getWritable;

  std::string m_fileName;
  std::string m_name;
  std::string m_path;
  std::shared_ptr<ParameterString> m_trigger;
};
} // namespace al

#endif // AL_DISKBUFFER_HPP
