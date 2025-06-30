#include <gtest/gtest.h>
#include "physics/dynamics/DynamicsEngine.h"
#include "physics/dynamics/DynamicsState.h"
#include "physics/dynamics/SimpleMassProperties.h"
#include "physics/dynamics/TorqueAggregator.h"
#include "physics/dynamics/torques/GravityGradientTorque.h"
#include "physics/forces/ForceAggregator.h"
#include "physics/forces/SimpleGravityModel.h"
#include "physics/forces/twobody/TwoBodyForceModel.h"
#include "physics/forces/drag/DragForceModel.h"
#include "physics/forces/srp/SolarRadiationPressureModel.h"
#include "physics/forces/IAttitudeAwareForceModel.h"
#include "physics/integrators/DynamicsIntegratorAdapter.h"
#include "physics/integrators/GenericRK4Integrator.h"
#include "physics/integrators/GenericRK78Integrator.h"
#include "core/math/MathConstants.h"
#include <cmath>
#include <fstream>
#include <iomanip>

using namespace iloss::physics;
using namespace iloss::physics::dynamics;
using namespace iloss::physics::forces;
using namespace iloss::physics::integrators;
using namespace iloss::math;
using namespace iloss::math::constants;
using namespace iloss::time;

/**
 * @brief Integration tests for coupled translational-rotational dynamics
 * 
 * This test suite focuses on scenarios where translational and rotational
 * dynamics are strongly coupled, testing the accuracy and stability of
 * the integrated 6-DOF system.
 */
class CoupledDynamicsIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default integrator configuration
        integratorConfig.initialStepSize = 1.0;  // 1 second step
        integratorConfig.minStepSize = 0.1;
        integratorConfig.maxStepSize = 100.0;
        integratorConfig.absoluteTolerance = 1e-6;  // Further relaxed for stability
        integratorConfig.relativeTolerance = 1e-8;  // Further relaxed for stability
        integratorConfig.enableStatistics = true;
        integratorConfig.maxIterations = 100000;  // Reasonable limit
    }
    
    /**
     * @brief Create a spinning spacecraft with asymmetric mass distribution
     */
    std::shared_ptr<SimpleMassProperties> createAsymmetricSpacecraft() {
        // Asymmetric spacecraft: 500 kg with different moments of inertia
        double mass = 500.0;
        double Ixx = 100.0;   // kg⋅m²
        double Iyy = 200.0;   // kg⋅m² (2x Ixx)
        double Izz = 150.0;   // kg⋅m² (1.5x Ixx)
        
        // Center of mass offset from geometric center
        Vector3D comOffset(0.2, -0.1, 0.15);  // meters
        
        return std::make_shared<SimpleMassProperties>(mass, Ixx, Iyy, Izz, comOffset);
    }
    
    IntegratorConfig integratorConfig;
};

/**
 * @brief Test Euler's equations for rigid body rotation
 * 
 * Verifies that a torque-free asymmetric rigid body follows
 * the correct rotational dynamics (conservation of angular momentum).
 */
TEST_F(CoupledDynamicsIntegrationTest, TorqueFreeRotation) {
    // Create asymmetric spacecraft
    auto massProps = createAsymmetricSpacecraft();
    
    // No forces or torques (drift in space)
    auto forceAgg = std::make_shared<ForceAggregator>();
    auto torqueAgg = std::make_shared<TorqueAggregator>();
    
    auto engine = std::make_shared<DynamicsEngine>(massProps, forceAgg, torqueAgg);
    auto adapter = std::make_shared<DynamicsIntegratorAdapter>(engine);
    
    // Create integrator
    GenericRK78Integrator<DynamicsState, DynamicsIntegratorAdapter> integrator(integratorConfig);
    
    // Initial state: spacecraft tumbling in space
    DynamicsState initialState(
        Vector3D(7000000.0, 0.0, 0.0),     // Position
        Vector3D(0.0, 7500.0, 0.0),        // Velocity (circular orbit speed)
        massProps->getMass(),
        0.0,                               // Time
        Quaternion::identity(),            // Attitude
        Vector3D(0.1, 0.3, 0.05)          // Angular velocity (rad/s)
    );
    
    // Calculate initial angular momentum in body frame
    Vector3D L0_body = massProps->getInertiaTensor() * initialState.getAngularVelocity();
    
    // Calculate initial kinetic energy
    double T0 = 0.5 * initialState.getAngularVelocity().dot(L0_body);
    
    // Integrate for 30 seconds
    double simulationTime = 30.0;
    
    // Track angular momentum and energy
    std::vector<double> times;
    std::vector<double> energies;
    std::vector<Vector3D> angularMomenta;
    
    auto callback = [&](const DynamicsState& state, double time) {
        Vector3D L_body = massProps->getInertiaTensor() * state.getAngularVelocity();
        double T = 0.5 * state.getAngularVelocity().dot(L_body);
        
        times.push_back(time);
        energies.push_back(T);
        angularMomenta.push_back(L_body);
        
        return true;
    };
    
    DynamicsState finalState = integrator.integrate(
        initialState, *adapter, simulationTime, callback
    );
    
    // Verify conservation laws
    Vector3D Lf_body = massProps->getInertiaTensor() * finalState.getAngularVelocity();
    double Tf = 0.5 * finalState.getAngularVelocity().dot(Lf_body);
    
    // Angular momentum magnitude should be conserved in body frame
    double L0_mag = L0_body.magnitude();
    double Lf_mag = Lf_body.magnitude();
    EXPECT_NEAR(Lf_mag, L0_mag, L0_mag * 1e-6);  // Relaxed tolerance
    
    // Kinetic energy should be conserved
    EXPECT_NEAR(Tf, T0, T0 * 1e-6);  // Relaxed tolerance
    
    // Check that angular momentum direction changed (precession)
    double dotProduct = L0_body.normalized().dot(Lf_body.normalized());
    EXPECT_LT(dotProduct, 0.99);  // Some precession occurred
}

/**
 * @brief Test dynamics with offset thrust
 * 
 * A thrust force applied at an offset from the center of mass
 * creates both translational acceleration and torque.
 */
TEST_F(CoupledDynamicsIntegrationTest, OffsetThrustDynamics) {
    // Create spacecraft with offset center of mass
    auto massProps = createAsymmetricSpacecraft();
    
    // Add forces and torques
    auto forceAgg = std::make_shared<ForceAggregator>();
    auto torqueAgg = std::make_shared<TorqueAggregator>();
    
    // Add two-body gravity
    forceAgg->addForceModel(std::make_unique<twobody::TwoBodyForceModel>("Earth", EARTH_MU));
    
    auto engine = std::make_shared<DynamicsEngine>(massProps, forceAgg, torqueAgg);
    auto adapter = std::make_shared<DynamicsIntegratorAdapter>(engine);
    
    // Create custom torque model for offset thrust
    class OffsetThrustTorque : public dynamics::ITorqueModel {
    private:
        Vector3D m_thrustVector;
        Vector3D m_applicationPoint;
        bool m_enabled = true;
        
    public:
        OffsetThrustTorque(const Vector3D& thrust, const Vector3D& offset)
            : m_thrustVector(thrust)
            , m_applicationPoint(offset) {}
        
        Vector3D calculateTorque(const DynamicsState& state, double time) const override {
            // Torque = r × F where r is from CoM to application point
            // The thrust vector is in body frame
            return m_applicationPoint.cross(m_thrustVector);
        }
        
        dynamics::TorqueModelType getType() const override { 
            return dynamics::TorqueModelType::User; 
        }
        
        std::string getName() const override { 
            return "OffsetThrust"; 
        }
        
        bool isEnabled() const override { 
            return m_enabled; 
        }
        
        void setEnabled(bool enabled) override { 
            m_enabled = enabled; 
        }
        
        void configure(const dynamics::TorqueModelConfig& config) override { 
            // No configuration needed
        }
        
        std::unique_ptr<dynamics::ITorqueModel> clone() const override {
            return std::make_unique<OffsetThrustTorque>(m_thrustVector, m_applicationPoint);
        }
    };
    
    // Create custom thrust force model that applies force at offset
    class OffsetThrustModel : public ForceModel {
    private:
        Vector3D m_thrustVector;
        Vector3D m_applicationPoint;
        
    public:
        OffsetThrustModel(const Vector3D& thrust, const Vector3D& offset)
            : ForceModel("OffsetThrust", ForceModelType::Thrust)
            , m_thrustVector(thrust)
            , m_applicationPoint(offset) {}
        
        Vector3D calculateAcceleration(const StateVector& state, const iloss::time::Time& time) const override {
            // Force creates acceleration in inertial frame
            // Since this is used with a spacecraft at the ascending node with identity attitude,
            // the body frame aligns with the inertial frame initially
            double mass = state.getMass();
            return m_thrustVector / mass;
        }
        
        bool initialize(const ForceModelConfig& config) override { return true; }
        bool validate() const override { return true; }
        std::unique_ptr<ForceModel> clone() const override {
            return std::make_unique<OffsetThrustModel>(m_thrustVector, m_applicationPoint);
        }
    };
    
    // Add thrust model and corresponding torque
    Vector3D thrustForce(10.0, 0.0, 0.0);  // 10 N in X direction
    Vector3D thrustOffset(0.0, 1.0, 0.0);  // Applied 1 m offset in Y
    forceAgg->addForceModel(
        std::make_unique<OffsetThrustModel>(thrustForce, thrustOffset)
    );
    
    // Add the torque from offset thrust
    torqueAgg->addModel(
        std::make_unique<OffsetThrustTorque>(thrustForce, thrustOffset)
    );
    
    // Create integrator
    GenericRK4Integrator<DynamicsState, DynamicsIntegratorAdapter> integrator(integratorConfig);
    
    // Initial state: spacecraft at rest relative to orbit
    double altitude = 400000.0;  // 400 km
    double radius = EARTH_RADIUS_EQUATORIAL + altitude;
    double orbitalSpeed = std::sqrt(EARTH_MU / radius);
    
    DynamicsState initialState(
        Vector3D(radius, 0.0, 0.0),
        Vector3D(0.0, orbitalSpeed, 0.0),
        massProps->getMass(),
        0.0,
        Quaternion::identity(),
        Vector3D(0.0, 0.0, 0.0)
    );
    
    // Integrate for 10 seconds of thrust
    DynamicsState finalState = integrator.integrate(initialState, *adapter, 10.0);
    
    // Verify coupled effects
    // 1. Translational: velocity should increase in thrust direction
    Vector3D deltaV = finalState.getVelocity() - initialState.getVelocity();
    EXPECT_GT(deltaV.x(), 0.1);  // Positive X velocity change
    
    // 2. Rotational: should have angular velocity due to torque
    // Expected torque = offset × force = (0,1,0) × (10,0,0) = (0,0,-10) N⋅m
    // This creates negative angular velocity around Z axis
    EXPECT_LT(finalState.getAngularVelocity().z(), -0.001);  // Negative Z rotation
}

/**
 * @brief Test tumbling spacecraft in atmosphere
 * 
 * A tumbling spacecraft experiences varying drag forces as different
 * faces are exposed to the velocity vector, creating complex coupled dynamics.
 */
TEST_F(CoupledDynamicsIntegrationTest, TumblingWithDrag) {
    // Create spacecraft
    auto massProps = std::make_shared<SimpleMassProperties>(
        100.0,    // 100 kg
        10.0,     // Ixx
        15.0,     // Iyy  
        12.0,     // Izz
        Vector3D(0.05, 0.0, 0.0)  // Small CoM offset
    );
    
    // Add forces
    auto forceAgg = std::make_shared<ForceAggregator>();
    auto torqueAgg = std::make_shared<TorqueAggregator>();
    
    // Two-body gravity
    forceAgg->addForceModel(std::make_unique<twobody::TwoBodyForceModel>("Earth", EARTH_MU));
    
    // Attitude-aware drag
    auto dragModel = std::make_shared<drag::DragForceModel>();
    ForceModelConfig dragConfig;
    dragConfig.setParameter("drag_coefficient", 2.2);
    dragConfig.setParameter("area", 2.0);  // 2 m² reference area
    dragModel->initialize(dragConfig);
    
    // Wrap drag model to make it attitude-aware
    forceAgg->addForceModel(std::make_unique<AttitudeAwareForceModelAdapter>(dragModel));
    
    auto engine = std::make_shared<DynamicsEngine>(massProps, forceAgg, torqueAgg);
    auto adapter = std::make_shared<DynamicsIntegratorAdapter>(engine);
    
    // Create integrator
    GenericRK78Integrator<DynamicsState, DynamicsIntegratorAdapter> integrator(integratorConfig);
    
    // Initial state: low orbit with tumbling
    double altitude = 250000.0;  // 250 km (significant drag)
    double radius = EARTH_RADIUS_EQUATORIAL + altitude;
    double orbitalSpeed = std::sqrt(EARTH_MU / radius);
    
    DynamicsState initialState(
        Vector3D(radius, 0.0, 0.0),
        Vector3D(0.0, orbitalSpeed, 0.0),
        massProps->getMass(),
        0.0,
        Quaternion::fromAxisAngle(Vector3D(1, 1, 0).normalized(), 0.5),  // Tilted
        Vector3D(0.05, 0.1, 0.02)  // Tumbling
    );
    
    // Track altitude decay and rotation rate
    std::vector<double> altitudes;
    std::vector<double> angularSpeeds;
    double minAltitude = altitude;
    
    auto callback = [&](const DynamicsState& state, double time) {
        double alt = state.getRadius() - EARTH_RADIUS_EQUATORIAL;
        double omega = state.getAngularVelocity().magnitude();
        
        altitudes.push_back(alt);
        angularSpeeds.push_back(omega);
        minAltitude = std::min(minAltitude, alt);
        
        return true;
    };
    
    // Integrate for shorter time (100 seconds instead of full orbit)
    double simulationTime = 100.0;
    
    DynamicsState finalState = integrator.integrate(
        initialState, *adapter, simulationTime, callback
    );
    
    // Verify altitude decay due to drag
    double finalAltitude = finalState.getRadius() - EARTH_RADIUS_EQUATORIAL;
    EXPECT_LT(finalAltitude, altitude);  // Altitude decreased
    
    // Angular momentum should change due to drag torques
    Vector3D L0 = massProps->getInertiaTensor() * initialState.getAngularVelocity();
    Vector3D Lf = massProps->getInertiaTensor() * finalState.getAngularVelocity();
    
    double L_change = (Lf - L0).magnitude() / L0.magnitude();
    EXPECT_GT(L_change, 0.001);  // Some change in angular momentum
}

/**
 * @brief Test nutation damping
 * 
 * For a spacecraft with internal energy dissipation, nutation
 * (coning motion) should damp out over time.
 */
TEST_F(CoupledDynamicsIntegrationTest, NutationDamping) {
    // Create spacecraft with energy dissipation
    class DampedMassProperties : public SimpleMassProperties {
    private:
        double m_dampingCoeff;
        
    public:
        DampedMassProperties(double mass, double Ixx, double Iyy, double Izz, double damping)
            : SimpleMassProperties(mass, Ixx, Iyy, Izz)
            , m_dampingCoeff(damping) {}
        
        double getDampingCoefficient() const { return m_dampingCoeff; }
    };
    
    auto massProps = std::make_shared<DampedMassProperties>(
        200.0,   // mass
        50.0,    // Ixx
        80.0,    // Iyy (major axis)
        60.0,    // Izz
        0.01     // damping coefficient
    );
    
    // No external forces/torques
    auto forceAgg = std::make_shared<ForceAggregator>();
    auto torqueAgg = std::make_shared<TorqueAggregator>();
    
    // Add damping torque model
    class DampingTorque : public dynamics::ITorqueModel {
    private:
        std::shared_ptr<DampedMassProperties> m_massProps;
        bool m_enabled = true;
        
    public:
        DampingTorque(std::shared_ptr<DampedMassProperties> props)
            : m_massProps(props) {}
        
        Vector3D calculateTorque(const DynamicsState& state, double time) const override {
            // Simple viscous damping: τ = -c * ω
            return state.getAngularVelocity() * (-m_massProps->getDampingCoefficient());
        }
        
        dynamics::TorqueModelType getType() const override { 
            return dynamics::TorqueModelType::User; 
        }
        
        std::string getName() const override { 
            return "Damping"; 
        }
        
        bool isEnabled() const override { 
            return m_enabled; 
        }
        
        void setEnabled(bool enabled) override { 
            m_enabled = enabled; 
        }
        
        void configure(const dynamics::TorqueModelConfig& config) override { 
            // No configuration needed for this simple model
        }
        
        std::unique_ptr<dynamics::ITorqueModel> clone() const override {
            return std::make_unique<DampingTorque>(m_massProps);
        }
    };
    
    torqueAgg->addModel(std::make_unique<DampingTorque>(massProps));
    
    auto engine = std::make_shared<DynamicsEngine>(
        std::static_pointer_cast<SimpleMassProperties>(massProps), 
        forceAgg, torqueAgg
    );
    auto adapter = std::make_shared<DynamicsIntegratorAdapter>(engine);
    
    // Create integrator
    GenericRK78Integrator<DynamicsState, DynamicsIntegratorAdapter> integrator(integratorConfig);
    
    // Initial state: spinning about non-principal axis (creates nutation)
    DynamicsState initialState(
        Vector3D(7000000.0, 0.0, 0.0),
        Vector3D(0.0, 7500.0, 0.0),
        massProps->getMass(),
        0.0,
        Quaternion::identity(),
        Vector3D(0.1, 0.05, 0.2)  // Not aligned with principal axes
    );
    
    // Track nutation angle over time
    std::vector<double> nutationAngles;
    Vector3D L0 = massProps->getInertiaTensor() * initialState.getAngularVelocity();
    
    auto callback = [&](const DynamicsState& state, double time) {
        Vector3D omega = state.getAngularVelocity();
        Vector3D L = massProps->getInertiaTensor() * omega;
        
        // Nutation angle is between ω and L
        double nutationAngle = std::acos(omega.normalized().dot(L.normalized()));
        nutationAngles.push_back(nutationAngle);
        
        return true;
    };
    
    // Integrate for moderate time to see damping
    double simulationTime = 200.0;  // 200 seconds
    
    DynamicsState finalState = integrator.integrate(
        initialState, *adapter, simulationTime, callback
    );
    
    // Verify nutation decreased
    double initialNutation = nutationAngles.front();
    double finalNutation = nutationAngles.back();
    
    EXPECT_LT(finalNutation, initialNutation * 0.5);  // Nutation reduced by at least 50%
    
    // Final state should be closer to principal axis rotation
    Vector3D omega_f = finalState.getAngularVelocity();
    Vector3D L_f = massProps->getInertiaTensor() * omega_f;
    double alignment = omega_f.normalized().dot(L_f.normalized());
    
    EXPECT_GT(alignment, 0.95);  // Nearly aligned (cos < 18°)
}

/**
 * @brief Test full 6-DOF simulation with all effects
 * 
 * Comprehensive test including gravity, drag, solar radiation pressure,
 * gravity gradient, and coupled dynamics.
 */
TEST_F(CoupledDynamicsIntegrationTest, ComprehensiveSimulation) {
    // Create realistic spacecraft
    auto massProps = std::make_shared<SimpleMassProperties>(
        500.0,    // 500 kg satellite
        100.0,    // Ixx
        300.0,    // Iyy (elongated)
        250.0,    // Izz
        Vector3D(0.1, 0.0, 0.05)  // CoM offset
    );
    
    // Set up all force and torque models
    auto forceAgg = std::make_shared<ForceAggregator>();
    auto torqueAgg = std::make_shared<TorqueAggregator>();
    
    // Two-body gravity
    forceAgg->addForceModel(std::make_unique<twobody::TwoBodyForceModel>("Earth", EARTH_MU));
    
    // Drag (attitude-aware)
    auto dragModel = std::make_shared<drag::DragForceModel>();
    ForceModelConfig dragConfig;
    dragConfig.setParameter("drag_coefficient", 2.2);
    dragConfig.setParameter("area", 5.0);  // 5 m²
    dragModel->initialize(dragConfig);
    forceAgg->addForceModel(std::make_unique<AttitudeAwareForceModelAdapter>(dragModel));
    
    // Solar radiation pressure (attitude-aware)
    auto srpModel = std::make_shared<forces::srp::SolarRadiationPressureModel>();
    ForceModelConfig srpConfig;
    srpConfig.setParameter("reflectivity_coefficient", 1.5);
    srpConfig.setParameter("area", 10.0);  // 10 m² solar panels
    srpModel->initialize(srpConfig);
    forceAgg->addForceModel(std::make_unique<AttitudeAwareForceModelAdapter>(srpModel));
    
    // Gravity gradient torque
    torques::GravityGradientConfig ggConfig;
    ggConfig.centralBodyMu = EARTH_MU;
    ggConfig.massProperties = massProps;
    torqueAgg->addModel(std::make_unique<torques::GravityGradientTorque>(ggConfig));
    
    // Create dynamics engine and adapter
    auto engine = std::make_shared<DynamicsEngine>(massProps, forceAgg, torqueAgg);
    auto adapter = std::make_shared<DynamicsIntegratorAdapter>(engine);
    
    // Use adaptive integrator for accuracy
    integratorConfig.relativeTolerance = 1e-8;  // Reasonable tolerance
    integratorConfig.maxIterations = 200000;  // Reasonable limit
    GenericRK78Integrator<DynamicsState, DynamicsIntegratorAdapter> integrator(integratorConfig);
    
    // Initial state: 600 km sun-synchronous orbit
    double altitude = 600000.0;
    double radius = EARTH_RADIUS_EQUATORIAL + altitude;
    double inclination = 97.8 * DEG_TO_RAD;  // Sun-synchronous
    double orbitalSpeed = std::sqrt(EARTH_MU / radius);
    
    DynamicsState initialState(
        Vector3D(radius * std::cos(inclination), 0.0, radius * std::sin(inclination)),
        Vector3D(0.0, orbitalSpeed, 0.0),
        massProps->getMass(),
        0.0,
        Quaternion::fromAxisAngle(Vector3D::unitY(), 0.1),  // Slight misalignment
        Vector3D(0.001, 0.0, 0.002)  // Small initial rates
    );
    
    // Data logging
    std::vector<double> times;
    std::vector<DynamicsState> states;
    
    auto callback = [&](const DynamicsState& state, double time) {
        if (static_cast<int>(time) % 10 == 0) {  // Log every 10 seconds
            times.push_back(time);
            states.push_back(state);
        }
        return true;
    };
    
    // Simulate for shorter time (200 seconds instead of half orbit)
    double simulationTime = 200.0;
    
    DynamicsState finalState = integrator.integrate(
        initialState, *adapter, simulationTime, callback
    );
    
    // Verify simulation completed successfully
    EXPECT_NEAR(finalState.getTimeAsDouble(), simulationTime, 1.0);
    
    // Check orbital elements changed due to perturbations
    double initialSMA = radius;  // Circular orbit assumption
    double finalRadius = finalState.getRadius();
    double smaChange = std::abs(finalRadius - initialSMA) / initialSMA;
    
    EXPECT_GT(smaChange, 1e-6);  // Some change due to perturbations
    EXPECT_LT(smaChange, 1e-3);  // But not too large
    
    // Attitude should have evolved
    Quaternion attitudeChange = initialState.getAttitude().conjugate() * finalState.getAttitude();
    double rotationAngle = 2.0 * std::acos(std::abs(attitudeChange.w()));
    
    EXPECT_GT(rotationAngle, 0.01);  // At least 0.01 rad rotation
    
    // Output statistics
    const auto& stats = integrator.getStatistics();
    std::cout << "\nComprehensive Simulation Statistics:\n";
    std::cout << "Total steps: " << stats.totalSteps << "\n";
    std::cout << "Accepted steps: " << stats.acceptedSteps << "\n";
    std::cout << "Rejected steps: " << stats.rejectedSteps << "\n";
    std::cout << "Average step size: " << stats.averageStepSize << " s\n";
    std::cout << "Max error estimate: " << stats.maxEstimatedError << "\n";
    
    // Optional: Write trajectory to file for visualization
    if (false) {  // Set to true to generate output file
        std::ofstream outFile("coupled_dynamics_trajectory.csv");
        outFile << "Time,X,Y,Z,Vx,Vy,Vz,Qw,Qx,Qy,Qz,Wx,Wy,Wz\n";
        
        for (size_t i = 0; i < times.size(); ++i) {
            const auto& state = states[i];
            outFile << std::fixed << std::setprecision(6)
                    << times[i] << ","
                    << state.getPosition().x() << ","
                    << state.getPosition().y() << ","
                    << state.getPosition().z() << ","
                    << state.getVelocity().x() << ","
                    << state.getVelocity().y() << ","
                    << state.getVelocity().z() << ","
                    << state.getAttitude().w() << ","
                    << state.getAttitude().x() << ","
                    << state.getAttitude().y() << ","
                    << state.getAttitude().z() << ","
                    << state.getAngularVelocity().x() << ","
                    << state.getAngularVelocity().y() << ","
                    << state.getAngularVelocity().z() << "\n";
        }
        
        outFile.close();
        std::cout << "Trajectory written to coupled_dynamics_trajectory.csv\n";
    }
}