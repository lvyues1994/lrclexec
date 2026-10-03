#pragma once
#include "GuiClient.h"
#include <memory>

class QMainWindow;
namespace simulation {
std::unique_ptr<QMainWindow> makeWorkbench(GuiClient &client, std::string const &modelFile);
} // namespace simulation
