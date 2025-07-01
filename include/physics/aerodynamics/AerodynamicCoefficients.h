#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <optional>

namespace iloss {
namespace physics {
namespace aerodynamics {

/**
 * @brief Structure to hold aerodynamic coefficients for a vehicle
 * 
 * This class stores the fundamental aerodynamic coefficients (drag, lift, moment, etc.)
 * and provides methods for accessing and validating them. Coefficients are typically
 * functions of Mach number and angle of attack, interpolated from tabulated data.
 * 
 * The coordinate system follows standard aerospace conventions:
 * - X: Forward (nose direction)
 * - Y: Right wing
 * - Z: Down
 * 
 * Coefficients:
 * - CD: Drag coefficient (force opposite to velocity)
 * - CL: Lift coefficient (force perpendicular to velocity in pitch plane)
 * - CY: Side force coefficient (force perpendicular to velocity in yaw plane)
 * - Cl: Rolling moment coefficient (about X axis)
 * - Cm: Pitching moment coefficient (about Y axis)
 * - Cn: Yawing moment coefficient (about Z axis)
 * 
 * @note All coefficients are dimensionless
 */
class AerodynamicCoefficients {
public:
    /**
     * @brief Default constructor initializes all coefficients to zero
     */
    AerodynamicCoefficients();

    /**
     * @brief Constructor with all coefficient values
     * 
     * @param cd Drag coefficient
     * @param cl Lift coefficient
     * @param cy Side force coefficient
     * @param croll Rolling moment coefficient
     * @param cpitch Pitching moment coefficient
     * @param cyaw Yawing moment coefficient
     * @throws std::invalid_argument if any coefficient is NaN or infinite
     */
    AerodynamicCoefficients(double cd, double cl, double cy, 
                           double croll, double cpitch, double cyaw);

    /**
     * @brief Constructor for force coefficients only (moments default to zero)
     * 
     * @param cd Drag coefficient
     * @param cl Lift coefficient
     * @param cy Side force coefficient (default 0)
     * @throws std::invalid_argument if any coefficient is NaN or infinite
     */
    explicit AerodynamicCoefficients(double cd, double cl, double cy = 0.0);

    // Getters for force coefficients
    double getDragCoefficient() const { return m_cd; }
    double getLiftCoefficient() const { return m_cl; }
    double getSideForceCoefficient() const { return m_cy; }
    
    // Getters for moment coefficients
    double getRollMomentCoefficient() const { return m_croll; }
    double getPitchMomentCoefficient() const { return m_cpitch; }
    double getYawMomentCoefficient() const { return m_cyaw; }

    // Setters with validation
    void setDragCoefficient(double cd);
    void setLiftCoefficient(double cl);
    void setSideForceCoefficient(double cy);
    void setRollMomentCoefficient(double croll);
    void setPitchMomentCoefficient(double cpitch);
    void setYawMomentCoefficient(double cyaw);

    /**
     * @brief Set all force coefficients at once
     * 
     * @param cd Drag coefficient
     * @param cl Lift coefficient
     * @param cy Side force coefficient
     * @throws std::invalid_argument if any coefficient is invalid
     */
    void setForceCoefficients(double cd, double cl, double cy);

    /**
     * @brief Set all moment coefficients at once
     * 
     * @param croll Rolling moment coefficient
     * @param cpitch Pitching moment coefficient
     * @param cyaw Yawing moment coefficient
     * @throws std::invalid_argument if any coefficient is invalid
     */
    void setMomentCoefficients(double croll, double cpitch, double cyaw);

    /**
     * @brief Check if coefficients are valid (finite and reasonable)
     * 
     * @return true if all coefficients are finite and within reasonable bounds
     */
    bool isValid() const;

    /**
     * @brief Get a string representation of the coefficients
     * 
     * @return String with all coefficient values
     */
    std::string toString() const;

    /**
     * @brief Scale all coefficients by a factor
     * 
     * Useful for Reynolds number corrections or uncertainty analysis
     * 
     * @param scaleFactor Factor to multiply all coefficients by
     * @return New AerodynamicCoefficients object with scaled values
     */
    AerodynamicCoefficients scale(double scaleFactor) const;

    /**
     * @brief Linearly interpolate between two coefficient sets
     * 
     * @param other The other coefficient set
     * @param factor Interpolation factor (0 = this, 1 = other)
     * @return Interpolated coefficients
     */
    AerodynamicCoefficients interpolate(const AerodynamicCoefficients& other, 
                                      double factor) const;

    // Operators
    bool operator==(const AerodynamicCoefficients& other) const;
    bool operator!=(const AerodynamicCoefficients& other) const;

    /**
     * @brief Add two coefficient sets
     * 
     * Useful for combining base coefficients with increments
     * 
     * @param other Coefficients to add
     * @return Sum of coefficients
     */
    AerodynamicCoefficients operator+(const AerodynamicCoefficients& other) const;
    AerodynamicCoefficients& operator+=(const AerodynamicCoefficients& other);

private:
    // Force coefficients
    double m_cd;     ///< Drag coefficient
    double m_cl;     ///< Lift coefficient
    double m_cy;     ///< Side force coefficient
    
    // Moment coefficients
    double m_croll;  ///< Rolling moment coefficient (Cl in standard notation)
    double m_cpitch; ///< Pitching moment coefficient (Cm in standard notation)
    double m_cyaw;   ///< Yawing moment coefficient (Cn in standard notation)

    /**
     * @brief Validate a coefficient value
     * 
     * @param value The coefficient value to check
     * @param name The name of the coefficient (for error messages)
     * @throws std::invalid_argument if value is NaN or infinite
     */
    void validateCoefficient(double value, const std::string& name) const;
};

/**
 * @brief Extended aerodynamic coefficients including stability derivatives
 * 
 * This class extends the basic coefficients with stability derivatives
 * that describe how coefficients change with angle of attack, sideslip,
 * and angular rates. These are essential for stability analysis and
 * accurate dynamic simulations.
 * 
 * Notation follows standard aerospace conventions:
 * - α (alpha): Angle of attack
 * - β (beta): Sideslip angle
 * - p, q, r: Body angular rates (roll, pitch, yaw)
 * 
 * Derivatives are partial derivatives, e.g., CLα = ∂CL/∂α
 */
class ExtendedAerodynamicCoefficients : public AerodynamicCoefficients {
public:
    /**
     * @brief Default constructor
     */
    ExtendedAerodynamicCoefficients();

    /**
     * @brief Constructor from base coefficients
     * 
     * @param base Base aerodynamic coefficients
     */
    explicit ExtendedAerodynamicCoefficients(const AerodynamicCoefficients& base);

    // Static stability derivatives (per radian)
    
    // Longitudinal derivatives
    double getCLalpha() const { return m_clAlpha; }  ///< Lift curve slope
    double getCDalpha() const { return m_cdAlpha; }  ///< Drag curve slope
    double getCMalpha() const { return m_cmAlpha; }  ///< Pitch stiffness
    
    // Lateral-directional derivatives
    double getCYbeta() const { return m_cyBeta; }    ///< Side force due to sideslip
    double getCLbeta() const { return m_clBeta; }    ///< Roll due to sideslip (dihedral effect)
    double getCNbeta() const { return m_cnBeta; }    ///< Yaw stiffness

    // Dynamic stability derivatives (per radian/second, normalized by V/c or V/b)
    
    // Pitch rate derivatives
    double getCLq() const { return m_clQ; }          ///< Lift due to pitch rate
    double getCMq() const { return m_cmQ; }          ///< Pitch damping
    
    // Roll rate derivatives
    double getCLp() const { return m_clP; }          ///< Roll damping
    double getCNp() const { return m_cnP; }          ///< Yaw due to roll rate
    
    // Yaw rate derivatives
    double getCLr() const { return m_clR; }          ///< Roll due to yaw rate
    double getCNr() const { return m_cnR; }          ///< Yaw damping

    // Setters for stability derivatives
    void setCLalpha(double value) { m_clAlpha = value; }
    void setCDalpha(double value) { m_cdAlpha = value; }
    void setCMalpha(double value) { m_cmAlpha = value; }
    void setCYbeta(double value) { m_cyBeta = value; }
    void setCLbeta(double value) { m_clBeta = value; }
    void setCNbeta(double value) { m_cnBeta = value; }
    void setCLq(double value) { m_clQ = value; }
    void setCMq(double value) { m_cmQ = value; }
    void setCLp(double value) { m_clP = value; }
    void setCNp(double value) { m_cnP = value; }
    void setCLr(double value) { m_clR = value; }
    void setCNr(double value) { m_cnR = value; }

    /**
     * @brief Apply increments due to angle changes and rates
     * 
     * @param alpha Angle of attack increment (radians)
     * @param beta Sideslip angle increment (radians)
     * @param p_normalized Roll rate normalized by V/b
     * @param q_normalized Pitch rate normalized by V/c
     * @param r_normalized Yaw rate normalized by V/b
     * @return Modified coefficients including all effects
     */
    AerodynamicCoefficients applyIncrements(double alpha, double beta,
                                          double p_normalized, 
                                          double q_normalized,
                                          double r_normalized) const;

    /**
     * @brief Check if any stability derivatives are defined
     * 
     * @return true if any derivative is non-zero
     */
    bool hasStabilityDerivatives() const;

private:
    // Longitudinal stability derivatives
    double m_clAlpha = 0.0;  ///< ∂CL/∂α (1/rad)
    double m_cdAlpha = 0.0;  ///< ∂CD/∂α (1/rad)
    double m_cmAlpha = 0.0;  ///< ∂CM/∂α (1/rad)
    
    // Lateral-directional stability derivatives
    double m_cyBeta = 0.0;   ///< ∂CY/∂β (1/rad)
    double m_clBeta = 0.0;   ///< ∂Cl/∂β (1/rad)
    double m_cnBeta = 0.0;   ///< ∂Cn/∂β (1/rad)
    
    // Dynamic derivatives
    double m_clQ = 0.0;      ///< ∂CL/∂(qc/V) (dimensionless)
    double m_cmQ = 0.0;      ///< ∂CM/∂(qc/V) (dimensionless)
    double m_clP = 0.0;      ///< ∂Cl/∂(pb/2V) (dimensionless)
    double m_cnP = 0.0;      ///< ∂Cn/∂(pb/2V) (dimensionless)
    double m_clR = 0.0;      ///< ∂Cl/∂(rb/2V) (dimensionless)
    double m_cnR = 0.0;      ///< ∂Cn/∂(rb/2V) (dimensionless)
};

} // namespace aerodynamics
} // namespace physics
} // namespace iloss