#include <sdu_controllers/utils/utility.hpp>

using namespace sdu_controllers::utils;

ConfigFolder::ConfigFolder(std::filesystem::path path)
{
  // Add the provided path to the list of configuration directories if it exists
  if (std::filesystem::exists(path) && std::filesystem::is_directory(path))
  {
    config_dirs_.push_back(path);
  }
}

std::filesystem::path ConfigFolder::get_default_config_path()
{
  // Return the first valid configuration directory from the list
  if (!config_dirs_.empty())
  {
    for (const auto &config_dir : config_dirs_)
    {
      if (!std::filesystem::exists(config_dir))
      {
        continue;
      }

      return config_dir;
    }
    throw std::runtime_error("No valid configuration directory found in the list.");
  }
  else
  {
    throw std::runtime_error("No configuration directories have been set.");
  }
}

std::filesystem::path ConfigFolder::find_config_file(const std::string &filename)
{
  for (const auto &config_dir : config_dirs_)
  {
    if (!std::filesystem::exists(config_dir))
    {
      continue;
    }

    // Search recursively through the directory and its subdirectories
    for (const auto &entry : std::filesystem::recursive_directory_iterator(config_dir))
    {
      if (entry.is_regular_file() && entry.path().filename() == filename)
      {
        return entry.path();
      }
    }
  }

  // File not found in any config directory
  return std::filesystem::path();
}

#ifdef PROJECT_SOURCE_DIR
std::vector<std::filesystem::path> ConfigFolder::config_dirs_({ std::filesystem::path(PROJECT_SOURCE_DIR) / "config" });
#else
std::vector<std::filesystem::path> ConfigFolder::config_dirs_;
#endif

