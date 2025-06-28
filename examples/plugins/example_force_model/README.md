# Example Force Model Plugin

This is a template plugin for ILOSS demonstrating how to create a custom force model plugin.

## Overview

This example plugin implements a simple constant acceleration force model that can be configured to apply a constant acceleration in the X, Y, and Z directions.

## Building the Plugin

### Standalone Build

```bash
mkdir build
cd build
cmake ..
make
```

### As Part of ILOSS Build

Add the following to your main CMakeLists.txt:
```cmake
add_subdirectory(examples/plugins/example_force_model)
```

## Plugin Structure

### Required Files

1. **Header File** (`ExampleForceModelPlugin.h`): Declares the plugin class
2. **Source File** (`ExampleForceModelPlugin.cpp`): Implements the plugin interface
3. **CMakeLists.txt**: Build configuration

### Required Methods

Every plugin must implement the `IPlugin` interface:

- `getMetadata()`: Returns plugin information
- `getCapabilities()`: Returns plugin capabilities
- `initialize()`: One-time initialization
- `activate()`: Start plugin functionality
- `deactivate()`: Stop plugin functionality
- `shutdown()`: Clean up resources
- `getState()`: Current plugin state
- `getLastError()`: Last error message
- `configure()`: Set configuration values
- `getConfiguration()`: Get configuration values

### Required Exports

Every plugin must export two C functions:

```cpp
// Factory function to create plugin instance
extern "C" std::unique_ptr<iloss::plugins::IPlugin> createPlugin();

// API version function
extern "C" const char* getPluginApiVersion();
```

Use the `ILOSS_PLUGIN_EXPORT_FUNCTIONS` macro to automatically generate these.

## Configuration

This example plugin supports the following configuration keys:

- `acceleration_x`: X-axis acceleration (m/s²)
- `acceleration_y`: Y-axis acceleration (m/s²)
- `acceleration_z`: Z-axis acceleration (m/s²)

## Usage Example

```cpp
// Load the plugin
auto& registry = PluginRegistry::getInstance();
std::string pluginId = registry.loadPlugin("example_force_model.so");

// Configure the plugin
registry.configurePlugin(pluginId, "acceleration_x", "0.1");
registry.configurePlugin(pluginId, "acceleration_y", "0.0");
registry.configurePlugin(pluginId, "acceleration_z", "0.0");

// Activate the plugin
registry.activatePlugin(pluginId);
```

## Creating Your Own Plugin

1. Copy this template directory
2. Rename the files and classes
3. Update the metadata in `getMetadata()`
4. Update the capabilities in `getCapabilities()`
5. Implement your plugin-specific functionality
6. Update the CMakeLists.txt with your plugin name

## Best Practices

1. **Error Handling**: Always set error messages when operations fail
2. **State Management**: Respect the plugin lifecycle states
3. **Thread Safety**: Make your plugin thread-safe if needed
4. **Resource Management**: Clean up resources in `shutdown()`
5. **Configuration Validation**: Validate all configuration values
6. **Logging**: Use the ILOSS logging system for diagnostics
7. **Dependencies**: Declare all plugin dependencies in metadata

## Plugin Lifecycle

```
UNLOADED -> LOADED -> INITIALIZED -> ACTIVE -> INITIALIZED -> LOADED -> UNLOADED
              ^           |             |           ^
              |           v             v           |
              +-------  ERROR  <--------+-----------+
```

## Debugging

Enable plugin debug logging:
```cpp
Logger::getInstance().setLevel("Plugin", LogLevel::DEBUG);
```

## Security Considerations

- Plugins run with the same privileges as ILOSS
- Validate all inputs
- Don't load plugins from untrusted sources
- Use the plugin validation features