#include <iostream>
#include "physics/forces/srp/SolarRadiationPressureModel.h"
#include "physics/forces/ForceModel.h"

using namespace iloss::physics::forces;
using namespace iloss::physics::forces::srp;

int main() {
    std::cout << "Testing SRP initialization..." << std::endl;
    
    SolarRadiationPressureModel srp;
    ForceModelConfig config;
    
    std::cout << "Setting parameters..." << std::endl;
    config.setParameter("reflectivity_coefficient", 1.8);
    config.setParameter("area", 20.0);
    
    // This is where it might hang
    std::cout << "Setting shadow_model string parameter..." << std::endl;
    config.setParameter("shadow_model", std::string("cylindrical"));
    
    std::cout << "Calling initialize..." << std::endl;
    bool result = srp.initialize(config);
    
    std::cout << "Initialize returned: " << result << std::endl;
    
    return 0;
}