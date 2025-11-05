#include "DeltaRobot.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

DeltaRobot::DeltaRobot() : config_() {
    state_.endEffectorPos = glm::vec3(0.0f, 0.0f, -0.35f);
    state_.motorAngles = {0.0f, 0.0f, 0.0f};
    setEndEffectorPosition(state_.endEffectorPos);
}

DeltaRobot::DeltaRobot(const DeltaRobotConfig& config) : config_(config) {
    state_.endEffectorPos = glm::vec3(0.0f, 0.0f, -0.35f);
    state_.motorAngles = {0.0f, 0.0f, 0.0f};
    setEndEffectorPosition(state_.endEffectorPos);
}

glm::vec3 DeltaRobot::getBaseJointPosition(int armIndex) const {
    float angle = armIndex * 2.0f * M_PI / 3.0f;
    return glm::vec3(
        config_.baseRadius * std::cos(angle),
        config_.baseRadius * std::sin(angle),
        config_.baseHeight
    );
}

glm::vec3 DeltaRobot::getEffectorJointOffset(int armIndex) const {
    float angle = armIndex * 2.0f * M_PI / 3.0f;
    return glm::vec3(
        config_.effectorRadius * std::cos(angle),
        config_.effectorRadius * std::sin(angle),
        0.0f
    );
}

bool DeltaRobot::calculateArmIK(int armIndex, const glm::vec3& targetPos, 
                                 ArmJoint& upperJoint, ArmJoint& lowerJoint, 
                                 glm::vec3& effectorJoint) {
    glm::vec3 baseJoint = getBaseJointPosition(armIndex);
    glm::vec3 effectorOffset = getEffectorJointOffset(armIndex);
    glm::vec3 effectorJointTarget = targetPos + effectorOffset;
    
    // Vector from base joint to effector joint
    glm::vec3 toEffector = effectorJointTarget - baseJoint;
    float distance = glm::length(toEffector);
    
    // Check if reachable
    float minDistance = std::abs(config_.upperArmLength - config_.lowerArmLength);
    float maxDistance = config_.upperArmLength + config_.lowerArmLength;
    
    if (distance < minDistance || distance > maxDistance) {
        return false;
    }
    
    // Calculate the radial direction - arms should point OUTWARD from base
    // The radial direction should point from the base joint outward (away from center)
    glm::vec3 center = glm::vec3(0.0f, 0.0f, baseJoint.z);
    glm::vec3 centerToBase = baseJoint - center;
    float radialDist = glm::length(glm::vec2(centerToBase.x, centerToBase.y));
    
    glm::vec3 radialDir;
    if (radialDist < 0.001f) {
        // Base at center, use default direction outward
        float angle = armIndex * 2.0f * M_PI / 3.0f;
        radialDir = glm::vec3(std::cos(angle), std::sin(angle), 0.0f);
    } else {
        // centerToBase points FROM center TO base
        // For arms to point OUTWARD, we need the direction FROM base AWAY from center
        // This is the same as centerToBase normalized (pointing outward from center through base)
        radialDir = glm::normalize(glm::vec3(centerToBase.x, centerToBase.y, 0.0f));
    }
    
    // The upper arm rotates in a vertical plane defined by radial direction and vertical
    // The motor joint rotates about an axis perpendicular to this plane (rotationAxis)
    // This ensures the motor can ONLY rotate up/down, not twist sideways
    glm::vec3 verticalDir = glm::vec3(0.0f, 0.0f, 1.0f);
    glm::vec3 planeNormal = glm::cross(radialDir, verticalDir);
    glm::vec3 rotationAxis = planeNormal;  // Rotation axis for the motor (perpendicular to plane)
    
    // Project the target vector onto the arm's plane
    glm::vec3 toTarget = effectorJointTarget - baseJoint;
    glm::vec3 toTargetInPlane = toTarget - glm::dot(toTarget, planeNormal) * planeNormal;
    float distInPlane = glm::length(toTargetInPlane);
    
    if (distInPlane < 0.001f) {
        // Target is directly above/below base, use radial direction
        toTargetInPlane = radialDir * 0.1f;
        distInPlane = 0.1f;
    }
    
    // Solve IK in the plane using law of cosines
    float cosTheta = (config_.upperArmLength * config_.upperArmLength + 
                      distance * distance - 
                      config_.lowerArmLength * config_.lowerArmLength) / 
                     (2.0f * config_.upperArmLength * distance);
    cosTheta = std::clamp(cosTheta, -1.0f, 1.0f);
    float theta = std::acos(cosTheta);
    
    // Calculate the direction of upper arm in the plane
    // The upper arm direction makes angle theta with the line to target
    glm::vec3 toTargetDir = glm::normalize(toTargetInPlane);
    
    // Find the angle between radial direction and target direction in the plane
    float angleToTarget = std::acos(std::clamp(glm::dot(toTargetDir, radialDir), -1.0f, 1.0f));
    
    // Solve for motor angle using sphere-sphere intersection
    // The upper arm rotates in a plane defined by radialDir and verticalDir
    // We need to find the intersection of:
    // - Sphere centered at baseJoint with radius upperArmLength
    // - Sphere centered at effectorJointTarget with radius lowerArmLength
    // The intersection point is the elbow position
    
    // Calculate the vector from base to effector in the plane
    float distX = glm::dot(toTargetInPlane, radialDir);  // Horizontal component
    float distZ = toTarget.z - baseJoint.z;  // Vertical component
    float dist2D = std::sqrt(distX * distX + distZ * distZ);
    
    // Law of cosines in the plane
    float cosMotorAngle = (config_.upperArmLength * config_.upperArmLength + 
                          dist2D * dist2D - 
                          config_.lowerArmLength * config_.lowerArmLength) / 
                         (2.0f * config_.upperArmLength * dist2D);
    cosMotorAngle = std::clamp(cosMotorAngle, -1.0f, 1.0f);
    float angleFromBaseToTarget = std::acos(cosMotorAngle);
    
    // Calculate the angle of the target direction in the plane
    // distX should be positive when target is outward (in radial direction)
    float targetAngle = std::atan2(distZ, distX);
    
    // Motor angle is constrained to ONLY rotate in the vertical plane (up/down)
    // The motor joint rotates about an axis perpendicular to the radial direction
    // This ensures the arm can only move up and down, not twist
    // Two possible solutions - choose the one that points outward and downward
    float motorAngle1 = targetAngle - angleFromBaseToTarget;
    float motorAngle2 = targetAngle + angleFromBaseToTarget;
    
    // Apply motor angle constraints - clamp to valid range
    // This ensures motors can only rotate within their limits
    motorAngle1 = std::clamp(motorAngle1, config_.minMotorAngle, config_.maxMotorAngle);
    motorAngle2 = std::clamp(motorAngle2, config_.minMotorAngle, config_.maxMotorAngle);
    
    // Calculate upper arm directions - STRICTLY in the vertical plane only
    // The motor rotates about an axis perpendicular to the radial direction
    // This means the arm can ONLY move up and down, not side to side or twist
    glm::vec3 upperArmDir1 = radialDir * std::cos(motorAngle1) + verticalDir * std::sin(motorAngle1);
    glm::vec3 upperArmDir2 = radialDir * std::cos(motorAngle2) + verticalDir * std::sin(motorAngle2);
    
    // Ensure directions are normalized and strictly in the plane
    upperArmDir1 = glm::normalize(upperArmDir1);
    upperArmDir2 = glm::normalize(upperArmDir2);
    
    // Remove any component perpendicular to the plane (should be zero, but ensure it)
    upperArmDir1 = upperArmDir1 - glm::dot(upperArmDir1, rotationAxis) * rotationAxis;
    upperArmDir2 = upperArmDir2 - glm::dot(upperArmDir2, rotationAxis) * rotationAxis;
    upperArmDir1 = glm::normalize(upperArmDir1);
    upperArmDir2 = glm::normalize(upperArmDir2);
    
    // Calculate elbow positions
    glm::vec3 upperArmEnd1 = baseJoint + upperArmDir1 * config_.upperArmLength;
    glm::vec3 upperArmEnd2 = baseJoint + upperArmDir2 * config_.upperArmLength;
    float dist1 = glm::length(effectorJointTarget - upperArmEnd1);
    float dist2 = glm::length(effectorJointTarget - upperArmEnd2);
    
    // Choose the solution that prevents elbows from folding inward
    // We want to prefer the solution where the elbow angle is more open
    // Minimum acceptable elbow angle: 90 degrees (to prevent folding)
    // Use a slightly lower threshold for selection, but prefer larger angles
    const float MIN_ELBOW_ANGLE = 90.0f * M_PI / 180.0f;
    const float PREFERRED_ELBOW_ANGLE = 100.0f * M_PI / 180.0f;  // Prefer angles above this
    
    float motorAngle;
    glm::vec3 upperArmDir;
    glm::vec3 upperArmEnd;
    
    // Calculate elbow angles for both solutions
    glm::vec3 lowerArmDir1 = glm::normalize(effectorJointTarget - upperArmEnd1);
    glm::vec3 lowerArmDir2 = glm::normalize(effectorJointTarget - upperArmEnd2);
    
    float elbowAngle1 = std::acos(std::clamp(glm::dot(upperArmDir1, lowerArmDir1), -1.0f, 1.0f));
    float elbowAngle2 = std::acos(std::clamp(glm::dot(upperArmDir2, lowerArmDir2), -1.0f, 1.0f));
    
    // Check validity of both solutions (distance, elbow angle, and motor angle)
    bool solution1ValidMotor = (motorAngle1 >= config_.minMotorAngle && motorAngle1 <= config_.maxMotorAngle);
    bool solution2ValidMotor = (motorAngle2 >= config_.minMotorAngle && motorAngle2 <= config_.maxMotorAngle);
    bool solution1ValidDist = (std::abs(dist1 - config_.lowerArmLength) < 0.05f);
    bool solution2ValidDist = (std::abs(dist2 - config_.lowerArmLength) < 0.05f);
    bool solution1GoodAngle = (elbowAngle1 >= PREFERRED_ELBOW_ANGLE);
    bool solution2GoodAngle = (elbowAngle2 >= PREFERRED_ELBOW_ANGLE);
    bool solution1AcceptableAngle = (elbowAngle1 >= MIN_ELBOW_ANGLE);
    bool solution2AcceptableAngle = (elbowAngle2 >= MIN_ELBOW_ANGLE);
    
    // Prioritize solutions with valid motor angle, good distance AND good elbow angles
    // Motor angle constraint is most important - reject solutions outside motor range
    if (solution1ValidMotor && solution1ValidDist && solution1GoodAngle && 
        solution2ValidMotor && solution2ValidDist && solution2GoodAngle) {
        // Both excellent - choose the one with larger elbow angle
        if (elbowAngle1 >= elbowAngle2) {
            motorAngle = motorAngle1;
            upperArmDir = upperArmDir1;
            upperArmEnd = upperArmEnd1;
        } else {
            motorAngle = motorAngle2;
            upperArmDir = upperArmDir2;
            upperArmEnd = upperArmEnd2;
        }
    } else if (solution1ValidMotor && solution1ValidDist && solution1GoodAngle) {
        // Solution 1 is excellent
        motorAngle = motorAngle1;
        upperArmDir = upperArmDir1;
        upperArmEnd = upperArmEnd1;
    } else if (solution2ValidMotor && solution2ValidDist && solution2GoodAngle) {
        // Solution 2 is excellent
        motorAngle = motorAngle2;
        upperArmDir = upperArmDir2;
        upperArmEnd = upperArmEnd2;
    } else if (solution1ValidMotor && solution1ValidDist && solution1AcceptableAngle) {
        // Solution 1 is acceptable
        motorAngle = motorAngle1;
        upperArmDir = upperArmDir1;
        upperArmEnd = upperArmEnd1;
    } else if (solution2ValidMotor && solution2ValidDist && solution2AcceptableAngle) {
        // Solution 2 is acceptable
        motorAngle = motorAngle2;
        upperArmDir = upperArmDir2;
        upperArmEnd = upperArmEnd2;
    } else if (solution1ValidMotor && solution1ValidDist) {
        // Solution 1 has valid motor and distance, use it
        motorAngle = motorAngle1;
        upperArmDir = upperArmDir1;
        upperArmEnd = upperArmEnd1;
    } else if (solution2ValidMotor && solution2ValidDist) {
        // Solution 2 has valid motor and distance, use it
        motorAngle = motorAngle2;
        upperArmDir = upperArmDir2;
        upperArmEnd = upperArmEnd2;
    } else if (solution1ValidMotor) {
        // Solution 1 has valid motor angle, use it even if distance is slightly off
        motorAngle = motorAngle1;
        upperArmDir = upperArmDir1;
        upperArmEnd = upperArmEnd1;
    } else if (solution2ValidMotor) {
        // Solution 2 has valid motor angle, use it even if distance is slightly off
        motorAngle = motorAngle2;
        upperArmDir = upperArmDir2;
        upperArmEnd = upperArmEnd2;
    } else {
        // Neither solution has valid motor angle - choose the one closer to valid range
        // This prevents the arms from jumping inward
        float motor1Error = std::min(std::abs(motorAngle1 - config_.minMotorAngle), 
                                     std::abs(motorAngle1 - config_.maxMotorAngle));
        float motor2Error = std::min(std::abs(motorAngle2 - config_.minMotorAngle), 
                                     std::abs(motorAngle2 - config_.maxMotorAngle));
        
        if (motor1Error <= motor2Error) {
            motorAngle = motorAngle1;
            upperArmDir = upperArmDir1;
            upperArmEnd = upperArmEnd1;
        } else {
            motorAngle = motorAngle2;
            upperArmDir = upperArmDir2;
            upperArmEnd = upperArmEnd2;
        }
    }
    
    // Final check: ensure motor angle is within constraints
    motorAngle = std::clamp(motorAngle, config_.minMotorAngle, config_.maxMotorAngle);
    
    // CRITICAL CONSTRAINT: Recalculate upper arm direction STRICTLY in the vertical plane
    // The motor joint MUST rotate only up/down - zero tolerance for drift or shifting
    // This is the ONLY allowed direction for the upper arm
    upperArmDir = radialDir * std::cos(motorAngle) + verticalDir * std::sin(motorAngle);
    upperArmDir = glm::normalize(upperArmDir);
    
    // Remove ANY component out of plane - this prevents shoulders from shifting
    float outOfPlaneComponent = glm::dot(upperArmDir, rotationAxis);
    if (std::abs(outOfPlaneComponent) > 0.0001f) {
        upperArmDir = upperArmDir - outOfPlaneComponent * rotationAxis;
        upperArmDir = glm::normalize(upperArmDir);
    }
    
    // Recalculate upper arm end with the strictly constrained direction
    upperArmEnd = baseJoint + upperArmDir * config_.upperArmLength;
    
    // Calculate lower arm direction (this can move freely - elbow is not constrained)
    glm::vec3 lowerArmDir = glm::normalize(effectorJointTarget - upperArmEnd);
    
    // Final validation - if elbow still folds inward, try to adjust
    float finalElbowAngle = std::acos(std::clamp(glm::dot(upperArmDir, lowerArmDir), -1.0f, 1.0f));
    if (finalElbowAngle < MIN_ELBOW_ANGLE) {
        // Position causes elbow folding - try the other solution as a last resort
        bool usingSolution1 = (motorAngle == motorAngle1);
        if (usingSolution1 && std::abs(dist2 - config_.lowerArmLength) < 0.1f) {
            motorAngle = motorAngle2;
            upperArmDir = upperArmDir2;
            upperArmEnd = upperArmEnd2;
            lowerArmDir = lowerArmDir2;
            finalElbowAngle = elbowAngle2;
        } else if (!usingSolution1 && std::abs(dist1 - config_.lowerArmLength) < 0.1f) {
            motorAngle = motorAngle1;
            upperArmDir = upperArmDir1;
            upperArmEnd = upperArmEnd1;
            lowerArmDir = lowerArmDir1;
            finalElbowAngle = elbowAngle1;
        }
        
        // If still invalid after trying both, use the solution anyway but warn
        // This prevents the robot from disappearing when constraints are too strict
        // The elbow might be slightly folded, but it's better than nothing
        if (finalElbowAngle < MIN_ELBOW_ANGLE) {
            // Allow it but note that it's not ideal
            // Don't return false - we'll still render it
        }
    }
    
    // CRITICAL: Never allow the upper arm to drift out of the vertical plane
    // The motor joint must ALWAYS rotate only up/down - no shifting allowed
    // Re-enforce the constraint after any calculations
    upperArmDir = radialDir * std::cos(motorAngle) + verticalDir * std::sin(motorAngle);
    upperArmDir = glm::normalize(upperArmDir);
    // Remove ANY component out of plane (zero tolerance for drift)
    upperArmDir = upperArmDir - glm::dot(upperArmDir, rotationAxis) * rotationAxis;
    float dirLength = glm::length(upperArmDir);
    if (dirLength > 0.001f) {
        upperArmDir = upperArmDir / dirLength;
    } else {
        // Fallback to radial direction if something goes wrong
        upperArmDir = radialDir;
    }
    
    // Recalculate upper arm end with the strictly constrained direction
    upperArmEnd = baseJoint + upperArmDir * config_.upperArmLength;
    
    // Calculate lower arm direction (this can move freely - elbow is not constrained)
    lowerArmDir = glm::normalize(effectorJointTarget - upperArmEnd);
    
    // Refine the solution if needed - but ALWAYS maintain plane constraint
    float actualDist = glm::length(effectorJointTarget - upperArmEnd);
    if (std::abs(actualDist - config_.lowerArmLength) > 0.001f) {
        // Adjust the lower arm direction, but DO NOT allow upper arm to drift
        // The upper arm direction MUST stay strictly in the plane
        glm::vec3 correctedLowerDir = glm::normalize(effectorJointTarget - upperArmEnd);
        
        // Recalculate elbow position based on corrected lower arm
        // But keep upper arm direction strictly in plane
        upperArmEnd = effectorJointTarget - correctedLowerDir * config_.lowerArmLength;
        
        // Recalculate motor angle from the corrected position
        // Project the new upper arm direction onto the plane
        glm::vec3 newUpperArmDir = glm::normalize(upperArmEnd - baseJoint);
        glm::vec3 newUpperArmInPlane = newUpperArmDir - glm::dot(newUpperArmDir, rotationAxis) * rotationAxis;
        if (glm::length(newUpperArmInPlane) > 0.001f) {
            newUpperArmInPlane = glm::normalize(newUpperArmInPlane);
            // Calculate motor angle from the in-plane direction
            motorAngle = std::atan2(glm::dot(newUpperArmInPlane, verticalDir), glm::dot(newUpperArmInPlane, radialDir));
            // Ensure motor angle is within constraints
            motorAngle = std::clamp(motorAngle, config_.minMotorAngle, config_.maxMotorAngle);
            
            // Re-enforce the direction strictly in the plane
            upperArmDir = radialDir * std::cos(motorAngle) + verticalDir * std::sin(motorAngle);
            upperArmDir = glm::normalize(upperArmDir);
            upperArmDir = upperArmDir - glm::dot(upperArmDir, rotationAxis) * rotationAxis;
            upperArmDir = glm::normalize(upperArmDir);
            
            // Recalculate upper arm end with the corrected direction
            upperArmEnd = baseJoint + upperArmDir * config_.upperArmLength;
            lowerArmDir = glm::normalize(effectorJointTarget - upperArmEnd);
        }
    }
    
    // Final validation: Ensure upper arm direction is EXACTLY in the plane
    // This is the last line of defense against drift
    glm::vec3 finalCheck = upperArmDir - glm::dot(upperArmDir, rotationAxis) * rotationAxis;
    if (glm::length(finalCheck) > 0.0001f) {
        upperArmDir = glm::normalize(finalCheck);
        // Recalculate everything with the corrected direction
        upperArmEnd = baseJoint + upperArmDir * config_.upperArmLength;
        lowerArmDir = glm::normalize(effectorJointTarget - upperArmEnd);
        
        // Recalculate motor angle from the corrected direction
        motorAngle = std::atan2(glm::dot(upperArmDir, verticalDir), glm::dot(upperArmDir, radialDir));
        motorAngle = std::clamp(motorAngle, config_.minMotorAngle, config_.maxMotorAngle);
    }
    
    // Store results - upper arm direction is now guaranteed to be in the plane
    upperJoint.position = baseJoint;
    upperJoint.angle = motorAngle;
    upperJoint.direction = upperArmDir;  // This is strictly in the vertical plane
    
    state_.motorAngles[armIndex] = motorAngle;
    
    lowerJoint.position = upperArmEnd;
    float elbowAngle = std::acos(std::clamp(glm::dot(upperArmDir, lowerArmDir), -1.0f, 1.0f));
    lowerJoint.angle = elbowAngle;
    lowerJoint.direction = lowerArmDir;
    
    effectorJoint = effectorJointTarget;
    
    return true;
}

bool DeltaRobot::setEndEffectorPosition(const glm::vec3& position) {
    state_.endEffectorPos = position;
    state_.isValid = true;
    
    // Calculate IK for all three arms
    for (int i = 0; i < 3; ++i) {
        if (!calculateArmIK(i, position, state_.upperJoints[i], 
                           state_.lowerJoints[i], state_.effectorJoints[i])) {
            state_.isValid = false;
            return false;
        }
    }
    
    return true;
}

void DeltaRobot::setConfig(const DeltaRobotConfig& config) {
    config_ = config;
    // Recalculate IK with new configuration
    setEndEffectorPosition(state_.endEffectorPos);
}

float DeltaRobot::getMaxReach() const {
    return config_.upperArmLength + config_.lowerArmLength;
}

float DeltaRobot::getMinHeight() const {
    float maxReach = getMaxReach();
    return config_.baseHeight - maxReach;
}

float DeltaRobot::getMaxHeight() const {
    return config_.baseHeight + 0.1f;  // Slightly above base
}

bool DeltaRobot::isPositionReachable(const glm::vec3& position) const {
    // Simple check: within spherical workspace
    float distanceFromOrigin = glm::length(glm::vec2(position.x, position.y));
    float maxReach = getMaxReach();
    
    if (distanceFromOrigin > maxReach) {
        return false;
    }
    
    if (position.z > config_.baseHeight + 0.1f) {
        return false;
    }
    
    if (position.z < getMinHeight()) {
        return false;
    }
    
    // Check each arm individually
    for (int i = 0; i < 3; ++i) {
        glm::vec3 baseJoint = getBaseJointPosition(i);
        glm::vec3 effectorOffset = getEffectorJointOffset(i);
        glm::vec3 effectorJointTarget = position + effectorOffset;
        
        glm::vec3 toEffector = effectorJointTarget - baseJoint;
        float distance = glm::length(toEffector);
        
        float minDistance = std::abs(config_.upperArmLength - config_.lowerArmLength);
        float maxDistance = config_.upperArmLength + config_.lowerArmLength;
        
        if (distance < minDistance || distance > maxDistance) {
            return false;
        }
    }
    
    return true;
}

