#ifndef AL_DISKBUFFER_HPP
#define AL_DISKBUFFER_HPP

#include <string>

#include "al/io/al_File.hpp"
#include "al/ui/al_Parameter.hpp"
#include "al/ui/al_ParameterServer.hpp"
#include "al_ext/tinc/al_BufferManager.hpp"
#include "nlohmann/json.hpp"

#include "al/graphics/al_Image.hpp"

namespace al {

template <class DataType>
class DiskBufferAbstract : public BufferManager<DataType> {
public:
  DiskBufferAbstract(std::string name, std::string fileName = "",
                     std::string path = "", uint16_t size = 2)
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

  virtual bool updateData(std::string filename = "") {
    if (filename.size() > 0) {
      m_fileName = filename;
    }
    std::ifstream file(m_path + m_fileName);
    if (file.good()) {
      auto buffer = getWritable();
      bool ret = parseFile(file, buffer);
      BufferManager<DataType>::doneWriting(buffer);
      return ret;
    } else {
      std::cerr << "Error code: " << strerror(errno);
      return false;
    }
  }

  // Careful, this is not thread safe. Needs to be called synchronously to any
  // process functions
  std::string getCurrentFileName() { return m_fileName; }

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

protected:
  virtual bool parseFile(std::ifstream &file,
                         std::shared_ptr<DataType> newData) = 0;

  // Make this function private as users should not have a way to make the
  // buffer writable. Data writing should be done by writing to the file.
  using BufferManager<DataType>::getWritable;

  std::string m_fileName;
  std::string m_name;
  std::string m_path;
  std::shared_ptr<ParameterString> m_trigger;
};

class JsonDiskBuffer : public DiskBufferAbstract<nlohmann::json> {
public:
  JsonDiskBuffer(std::string name, std::string fileName = "",
                 std::string path = "", uint16_t size = 2)
      : DiskBufferAbstract<nlohmann::json>(name, fileName, path, size) {}

protected:
  virtual bool parseFile(std::ifstream &file,
                         std::shared_ptr<nlohmann::json> newData) {

    //    try {
    *newData = nlohmann::json::parse(file);

    return true;
    //    } catch () {
    //      std::cerr << "ERROR: parsing file. Increase cache on writing side."
    //                << std::endl;
    //      return false;
    //    }
  }
};

class ImageDiskBuffer : public DiskBufferAbstract<Image> {
public:
  ImageDiskBuffer(std::string name, std::string fileName = "",
                  std::string path = "", uint16_t size = 2)
      : DiskBufferAbstract<Image>(name, fileName, path, size) {}
  bool updateData(std::string filename = "") override {
    if (filename.size() > 0) {
      m_fileName = filename;
    }
    auto buffer = getWritable();
    if (buffer->load(m_path + m_fileName)) {

      BufferManager<Image>::doneWriting(buffer);
      return true;
    } else {
      std::cerr << "Error reading Image: " << m_path + m_fileName << std::endl;
      return false;
    }
  }

protected:
  bool parseFile(std::ifstream &file, std::shared_ptr<Image> newData) override {
    return true;
  }
};

} // namespace al

#endif // AL_DISKBUFFER_HPP
