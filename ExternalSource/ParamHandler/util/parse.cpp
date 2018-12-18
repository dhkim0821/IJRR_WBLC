#include <fstream>
#include <iostream>
#include <vector>

#include "dynacore_yaml-cpp/eventhandler.h"
#include "dynacore_yaml-cpp/yaml.h"  // IWYU pragma: keep

struct Params {
  bool hasFile;
  std::string fileName;
};

Params ParseArgs(int argc, char** argv) {
  Params p;

  std::vector<std::string> args(argv + 1, argv + argc);

  return p;
}

class NullEventHandler : public dynacore_YAML::EventHandler {
 public:
  virtual void OnDocumentStart(const dynacore_YAML::Mark&) {}
  virtual void OnDocumentEnd() {}

  virtual void OnNull(const dynacore_YAML::Mark&, dynacore_YAML::anchor_t) {}
  virtual void OnAlias(const dynacore_YAML::Mark&, dynacore_YAML::anchor_t) {}
  virtual void OnScalar(const dynacore_YAML::Mark&, const std::string&, dynacore_YAML::anchor_t,
                        const std::string&) {}

  virtual void OnSequenceStart(const dynacore_YAML::Mark&, const std::string&,
                               dynacore_YAML::anchor_t) {}
  virtual void OnSequenceEnd() {}

  virtual void OnMapStart(const dynacore_YAML::Mark&, const std::string&,
                          dynacore_YAML::anchor_t) {}
  virtual void OnMapEnd() {}
};

void parse(std::istream& input) {
  try {
    dynacore_YAML::Node doc = dynacore_YAML::Load(input);
    std::cout << doc << "\n";
  } catch (const dynacore_YAML::Exception& e) {
    std::cerr << e.what() << "\n";
  }
}

int main(int argc, char** argv) {
  Params p = ParseArgs(argc, argv);

  if (argc > 1) {
    std::ifstream fin;
    fin.open(argv[1]);
    parse(fin);
  } else {
    parse(std::cin);
  }

  return 0;
}
