#include "physics/forces/ForceModelRegistry.h"
// Include only the models that are currently implemented
// SimpleGravityModel is commented out as it might be used for testing only
// #include "physics/forces/SimpleGravityModel.h"
#include "physics/forces/twobody/TwoBodyForceModel.h"
#include "physics/forces/gravity/EarthGravityModel.h"
#include "physics/forces/thirdbody/ThirdBodyForceModel.h"
#include "physics/forces/drag/DragForceModel.h"
#include "physics/forces/srp/SolarRadiationPressureModel.h"
#include "core/logging/Logger.h"

namespace iloss {
namespace physics {
namespace forces {

/**
 * @brief Register all standard force models with the registry
 * 
 * This function is called during initialization to register
 * all built-in force models with their factory functions.
 */
void registerStandardForceModels() {
    auto& registry = ForceModelRegistry::getInstance();
    
    LOG_INFO("ForceModelRegistry", "Registering standard force models");
    
    // Register Simple Gravity Model (for testing)
    // Commented out for now - uncomment when SimpleGravityModel is needed
    /*
    {
        ForceModelInfo info;
        info.type = ForceModelType::TwoBody;
        info.typeName = "SimpleGravity";
        info.description = "Simple point-mass gravitational model";
        info.requiredParams = {};
        info.optionalParams = {"mu", "central_body"};
        info.factory = [](const std::string& name) -> std::unique_ptr<ForceModel> {
            return std::make_unique<SimpleGravityModel>(name);
        };
        
        registry.registerForceModel(ForceModelType::TwoBody, info);
    }
    */
    
    // Register Two-Body Force Model
    {
        ForceModelInfo info;
        info.type = ForceModelType::TwoBody;
        info.typeName = "TwoBody";
        info.description = "Classical two-body gravitational force model";
        info.requiredParams = {};
        info.optionalParams = {"mu", "central_body"};
        info.factory = [](const std::string& name) -> std::unique_ptr<ForceModel> {
            return std::make_unique<twobody::TwoBodyForceModel>(name);
        };
        
        // Note: We're overwriting the SimpleGravity registration here
        // In a real system, we might want different types or a way to select
        registry.registerForceModel(ForceModelType::TwoBody, info);
    }
    
    // Register Earth Gravity Model
    {
        ForceModelInfo info;
        info.type = ForceModelType::GravityField;
        info.typeName = "EarthGravity";
        info.description = "Earth gravity field with spherical harmonics";
        info.requiredParams = {};
        info.optionalParams = {"degree", "order", "coefficient_file", "include_tides"};
        info.factory = [](const std::string& name) -> std::unique_ptr<ForceModel> {
            return std::make_unique<EarthGravityModel>(name);
        };
        
        registry.registerForceModel(ForceModelType::GravityField, info);
    }
    
    // Register Third-Body Force Model
    {
        ForceModelInfo info;
        info.type = ForceModelType::ThirdBody;
        info.typeName = "ThirdBody";
        info.description = "Third-body gravitational perturbations (Sun, Moon, planets)";
        info.requiredParams = {};
        info.optionalParams = {"bodies", "central_body", "cache_duration", "enable_all", "custom_bodies"};
        info.factory = [](const std::string& name) -> std::unique_ptr<ForceModel> {
            return std::make_unique<thirdbody::ThirdBodyForceModel>(name);
        };
        
        registry.registerForceModel(ForceModelType::ThirdBody, info);
    }
    
    // Register Atmospheric Drag Model
    {
        ForceModelInfo info;
        info.type = ForceModelType::Drag;
        info.typeName = "AtmosphericDrag";
        info.description = "Atmospheric drag force with multiple atmosphere models";
        info.requiredParams = {};
        info.optionalParams = {"drag_coefficient", "area", "ballistic_coefficient", 
                              "enable_atmospheric_rotation", "enable_wind", "wind_velocity",
                              "atmosphere_model"};
        info.factory = [](const std::string& /*name*/) -> std::unique_ptr<ForceModel> {
            return std::make_unique<drag::DragForceModel>();
        };
        
        registry.registerForceModel(ForceModelType::Drag, info);
    }
    
    // Register Solar Radiation Pressure Model
    {
        ForceModelInfo info;
        info.type = ForceModelType::SolarRadiation;
        info.typeName = "SolarRadiationPressure";
        info.description = "Solar radiation pressure with conical shadow model";
        info.requiredParams = {};
        info.optionalParams = {"reflectivity_coefficient", "area", "area_to_mass_ratio",
                              "shadow_model", "enable_flux_variation", "solar_flux_au",
                              "speed_of_light", "include_moon_shadow", "surface_normal"};
        info.factory = [](const std::string& /*name*/) -> std::unique_ptr<ForceModel> {
            return std::make_unique<srp::SolarRadiationPressureModel>();
        };
        
        registry.registerForceModel(ForceModelType::SolarRadiation, info);
    }
    
    // Future models to be registered:
    // - Earth Radiation Pressure
    // - Relativistic Corrections
    // - Thrust Models
    
    LOG_INFO("ForceModelRegistry", "Completed registration of standard force models");
}

} // namespace forces
} // namespace physics
} // namespace iloss