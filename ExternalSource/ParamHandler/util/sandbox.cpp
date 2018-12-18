#include <iostream>

#include "dynacore_yaml-cpp/emitterstyle.h"
#include "dynacore_yaml-cpp/eventhandler.h"
#include "dynacore_yaml-cpp/yaml.h"  // IWYU pragma: keep

class NullEventHandler : public dynacore_YAML::EventHandler {
 public:
  typedef dynacore_YAML::Mark Mark;
  typedef dynacore_YAML::anchor_t anchor_t;

  NullEventHandler() {}

  virtual void OnDocumentStart(const Mark&) {}
  virtual void OnDocumentEnd() {}
  virtual void OnNull(const Mark&, anchor_t) {}
  virtual void OnAlias(const Mark&, anchor_t) {}
  virtual void OnScalar(const Mark&, const std::string&, anchor_t,
                        const std::string&) {}
  virtual void OnSequenceStart(const Mark&, const std::string&, anchor_t,
                               dynacore_YAML::EmitterStyle::value style) {}
  virtual void OnSequenceEnd() {}
  virtual void OnMapStart(const Mark&, const std::string&, anchor_t,
                          dynacore_YAML::EmitterStyle::value style) {}
  virtual void OnMapEnd() {}
};

int main() {
  dynacore_YAML::Node root;

  for (;;) {
    dynacore_YAML::Node node;
    root = node;
  }
  return 0;
}
