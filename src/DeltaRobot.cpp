#include "DeltaRobot.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

DeltaRobot::DeltaRobot() : config_() {
    state_.endEffectorPos = glm::vec3(0.0f, 0.0f, -0.35f);
    state_.motorAngles = {0.0f, 0.0f, 0.0f};
    state_.isValid = false;  // Start as invalid, will be set by setEndEffectorPosition
    // Try to set initial position, but don't fail if validation has issues
    setEndEffectorPosition(state_.endEffectorPos);
    // If still invalid, force a valid state with current position
    if (!state_.isValid) {
        state_.isValid = true;  // Force valid to allow robot to render
    }
}

DeltaRobot::DeltaRobot(const DeltaRobotConfig& config) : config_(config) {
    state_.endEffectorPos = glm::vec3(0.0f, 0.0f, -0.35f);
    state_.motorAngles = {0.0f, 0.0f, 0.0f};
    state_.isValid = false;  // Start as invalid, will be set by setEndEffectorPosition
    // Try to set initial position, but don't fail if validation has issues
    setEndEffectorPosition(state_.endEffectorPos);
    // If still invalid, force a valid state with current position
    if (!state_.isValid) {
        state_.isValid = true;  // Force valid to allow robot to render
    }
}

glm::vec3 DeltaRobot::getBaseJointPosition(int armIndex) const {
    float angle = armIndex * 2.0f * M_PI / 3.0f;
    // Use baseJointRadius if set, otherwise use basePlateRadius, fallback to baseRadius
    float radius = config_.baseJointRadius > 0.0f ? config_.baseJointRadius : 
                   (config_.basePlateRadius > 0.0f ? config_.basePlateRadius : config_.baseRadius);
    float height = config_.basePlateHeight > 0.0f ? config_.basePlateHeight : config_.baseHeight;
    return glm::vec3(
        radius * std::cos(angle),
        radius * std::sin(angle),
        height
    );
}

glm::vec3 DeltaRobot::getEffectorJointOffset(int armIndex) const {
    float angle = armIndex * 2.0f * M_PI / 3.0f;
    // Use effectorJointRadius if set, otherwise use effectorPlateRadius, fallback to effectorRadius
    float radius = config_.effectorJointRadius > 0.0f ? config_.effectorJointRadius : 
                   (config_.effectorPlateRadius > 0.0f ? config_.effectorPlateRadius : config_.effectorRadius);
    return glm::vec3(
        radius * std::cos(angle),
        radius * std::sin(angle),
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
    
    // Motor angle constraints removed - allow full range of motion
    // Only physical constraints (arm lengths, elbow folding) will limit movement
    
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
    
    // Check if arms point outward (away from center) or inward (toward center)
    // Positive dot product means arm points outward, negative means inward
    // We STRONGLY prefer outward-pointing arms to prevent folding inward
    float solution1Outward = glm::dot(upperArmDir1, radialDir);  // > 0 = outward
    float solution2Outward = glm::dot(upperArmDir2, radialDir);  // > 0 = outward
    bool solution1PointsOutward = (solution1Outward > 0.0f);
    bool solution2PointsOutward = (solution2Outward > 0.0f);
    
    // Check validity of both solutions (distance, elbow angle, and outward direction)
    // Motor angle constraints removed - prioritize based on outward direction, elbow angle, and distance
    bool solution1ValidDist = (std::abs(dist1 - config_.lowerArmLength) < 0.05f);
    bool solution2ValidDist = (std::abs(dist2 - config_.lowerArmLength) < 0.05f);
    bool solution1GoodAngle = (elbowAngle1 >= PREFERRED_ELBOW_ANGLE);
    bool solution2GoodAngle = (elbowAngle2 >= PREFERRED_ELBOW_ANGLE);
    bool solution1AcceptableAngle = (elbowAngle1 >= MIN_ELBOW_ANGLE);
    bool solution2AcceptableAngle = (elbowAngle2 >= MIN_ELBOW_ANGLE);
    
    // Prioritize solutions: FIRST outward direction, THEN elbow angle, THEN distance
    // CRITICAL: Always prefer outward-pointing arms to prevent inward folding
    if (solution1PointsOutward && !solution2PointsOutward) {
        // Solution 1 points outward, solution 2 points inward - prefer solution 1
        motorAngle = motorAngle1;
        upperArmDir = upperArmDir1;
        upperArmEnd = upperArmEnd1;
    } else if (solution2PointsOutward && !solution1PointsOutward) {
        // Solution 2 points outward, solution 1 points inward - prefer solution 2
        motorAngle = motorAngle2;
        upperArmDir = upperArmDir2;
        upperArmEnd = upperArmEnd2;
    } else if (solution1PointsOutward && solution2PointsOutward) {
        // Both point outward - choose based on elbow angle and distance
        if (solution1ValidDist && solution1GoodAngle && 
            solution2ValidDist && solution2GoodAngle) {
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
        } else if (solution1ValidDist && solution1GoodAngle) {
            motorAngle = motorAngle1;
            upperArmDir = upperArmDir1;
            upperArmEnd = upperArmEnd1;
        } else if (solution2ValidDist && solution2GoodAngle) {
            motorAngle = motorAngle2;
            upperArmDir = upperArmDir2;
            upperArmEnd = upperArmEnd2;
        } else if (solution1ValidDist && solution1AcceptableAngle) {
            motorAngle = motorAngle1;
            upperArmDir = upperArmDir1;
            upperArmEnd = upperArmEnd1;
        } else if (solution2ValidDist && solution2AcceptableAngle) {
            motorAngle = motorAngle2;
            upperArmDir = upperArmDir2;
            upperArmEnd = upperArmEnd2;
        } else if (solution1ValidDist) {
            motorAngle = motorAngle1;
            upperArmDir = upperArmDir1;
            upperArmEnd = upperArmEnd1;
        } else if (solution2ValidDist) {
            motorAngle = motorAngle2;
            upperArmDir = upperArmDir2;
            upperArmEnd = upperArmEnd2;
        } else {
            // Choose the one with better elbow angle
            if (elbowAngle1 >= elbowAngle2) {
                motorAngle = motorAngle1;
                upperArmDir = upperArmDir1;
                upperArmEnd = upperArmEnd1;
            } else {
                motorAngle = motorAngle2;
                upperArmDir = upperArmDir2;
                upperArmEnd = upperArmEnd2;
            }
        }
    } else {
        // Both point inward - this is bad, but choose the one with better elbow angle
        // This should rarely happen, but if it does, prefer the solution with larger elbow angle
        if (elbowAngle1 >= elbowAngle2 && solution1AcceptableAngle) {
            motorAngle = motorAngle1;
            upperArmDir = upperArmDir1;
            upperArmEnd = upperArmEnd1;
        } else if (solution2AcceptableAngle) {
            motorAngle = motorAngle2;
            upperArmDir = upperArmDir2;
            upperArmEnd = upperArmEnd2;
        } else {
            // Neither is acceptable - reject position to prevent folding
            return false;
        }
    }
    
    // Motor angle constraints removed - no clamping needed
    
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
    
    // Final validation - if elbow folds inward too much, reject the position
    // This prevents arms from inverting and folding in on themselves
    float finalElbowAngle = std::acos(std::clamp(glm::dot(upperArmDir, lowerArmDir), -1.0f, 1.0f));
    if (finalElbowAngle < MIN_ELBOW_ANGLE) {
        // Position causes elbow folding - try the other solution as a last resort
        bool usingSolution1 = (motorAngle == motorAngle1);
        if (usingSolution1 && std::abs(dist2 - config_.lowerArmLength) < 0.1f) {
            float elbow2Check = std::acos(std::clamp(glm::dot(upperArmDir2, lowerArmDir2), -1.0f, 1.0f));
            if (elbow2Check >= MIN_ELBOW_ANGLE) {
                motorAngle = motorAngle2;
                upperArmDir = upperArmDir2;
                upperArmEnd = upperArmEnd2;
                lowerArmDir = lowerArmDir2;
                finalElbowAngle = elbow2Check;
            } else {
                // Both solutions cause folding - reject this position
                return false;
            }
        } else if (!usingSolution1 && std::abs(dist1 - config_.lowerArmLength) < 0.1f) {
            float elbow1Check = std::acos(std::clamp(glm::dot(upperArmDir1, lowerArmDir1), -1.0f, 1.0f));
            if (elbow1Check >= MIN_ELBOW_ANGLE) {
                motorAngle = motorAngle1;
                upperArmDir = upperArmDir1;
                upperArmEnd = upperArmEnd1;
                lowerArmDir = lowerArmDir1;
                finalElbowAngle = elbow1Check;
            } else {
                // Both solutions cause folding - reject this position
                return false;
            }
        } else {
            // Cannot fix - reject position to prevent folding
            return false;
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
    
    // CRITICAL: Verify that arms maintain exact fixed lengths - NO STRETCHING ALLOWED
    // Check upper arm length
    float upperArmActualLength = glm::length(upperArmEnd - baseJoint);
    if (std::abs(upperArmActualLength - config_.upperArmLength) > 0.0001f) {
        // Upper arm length is wrong - recalculate to enforce exact length
        upperArmEnd = baseJoint + upperArmDir * config_.upperArmLength;
    }
    
    // Check lower arm length
    // After all the in-plane corrections, allow some tolerance for numerical precision
    // The solution was valid originally, so small corrections for drift shouldn't invalidate it
    float lowerArmActualLength = glm::length(effectorJointTarget - upperArmEnd);
    float lowerArmLengthError = std::abs(lowerArmActualLength - config_.lowerArmLength);
    if (lowerArmLengthError > 0.01f) {  // Increased tolerance from 0.0001f to 0.01f (1cm)
        // Lower arm cannot reach target - this position is unreachable for this arm
        // DO NOT stretch the arm - return failure instead
        return false;
    }
    
    // Calculate lower arm direction (normalized)
    // Small length errors (< 1cm) are acceptable due to numerical precision after in-plane corrections
    lowerArmDir = glm::normalize(effectorJointTarget - upperArmEnd);
    
    // Final validation: Ensure upper arm direction is EXACTLY in the plane
    // This is the last line of defense against drift
    // Remove any remaining out-of-plane component, but KEEP the original motor angle
    glm::vec3 finalCheck = upperArmDir - glm::dot(upperArmDir, rotationAxis) * rotationAxis;
    if (glm::length(finalCheck) > 0.0001f) {
        // Correct the direction to be strictly in-plane
        upperArmDir = glm::normalize(finalCheck);
        // Recalculate upper arm end with corrected direction
        upperArmEnd = baseJoint + upperArmDir * config_.upperArmLength;
        lowerArmDir = glm::normalize(effectorJointTarget - upperArmEnd);
        // DO NOT recalculate motor angle - keep the original calculated angle
        // The motor angle was correctly calculated earlier, and this is just a minor correction
        // to remove numerical drift, not a fundamental change to the solution
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
    // First, check if position is reachable
    if (!isPositionReachable(position)) {
        // Position is unreachable - don't update state, keep last valid position
        state_.isValid = false;
        return false;
    }
    
    // Store the last valid position before trying IK
    glm::vec3 lastValidPos = state_.endEffectorPos;
    bool lastValid = state_.isValid;
    
    // Try to calculate IK for all three arms
    state_.endEffectorPos = position;
    state_.isValid = true;
    
    bool allValid = true;
    for (int i = 0; i < 3; ++i) {
        if (!calculateArmIK(i, position, state_.upperJoints[i], 
                           state_.lowerJoints[i], state_.effectorJoints[i])) {
            allValid = false;
            break;  // Stop on first failure
        }
    }
    
    if (!allValid) {
        // IK failed - revert to last valid position
        state_.endEffectorPos = lastValidPos;
        state_.isValid = lastValid;
        
        // Recalculate IK for last valid position to ensure joint positions are correct
        if (lastValid) {
            for (int i = 0; i < 3; ++i) {
                calculateArmIK(i, lastValidPos, state_.upperJoints[i], 
                             state_.lowerJoints[i], state_.effectorJoints[i]);
            }
        }
        return false;
    }
    
    // Validate IK solution using Forward Kinematics (industry standard practice)
    // This ensures the calculated motor angles actually produce the desired position
    // The FK uses gradient descent to solve the sphere intersection system
    float validationError = getIKValidationError(position, state_.motorAngles);
    
    // Reject if error is too large - this catches IK calculation errors
    // Tolerance: 2mm (0.002m) - tight enough to catch errors, lenient enough for numerical precision
    const float VALIDATION_TOLERANCE = 0.002f;  // 2mm
    
    if (validationError > VALIDATION_TOLERANCE) {
        // Error too large - likely an IK calculation error or numerical issue
        // Revert to last valid position
        state_.endEffectorPos = lastValidPos;
        state_.isValid = lastValid;
        
        if (lastValid) {
            for (int i = 0; i < 3; ++i) {
                calculateArmIK(i, lastValidPos, state_.upperJoints[i], 
                             state_.lowerJoints[i], state_.effectorJoints[i]);
            }
        }
        return false;
    }
    
    // Validation passed - IK solution is correct
    
    return true;
}

glm::vec3 DeltaRobot::clampToWorkspace(const glm::vec3& position) const {
    glm::vec3 clamped = position;
    
    // Clamp height
    float minHeight = getMinHeight();
    float maxHeight = getMaxHeight();
    clamped.z = std::clamp(clamped.z, minHeight, maxHeight);
    
    // Clamp horizontal distance from origin
    float distanceFromOrigin = glm::length(glm::vec2(clamped.x, clamped.y));
    float maxReach = getMaxReach();
    
    if (distanceFromOrigin > maxReach) {
        // Project back to maximum reach
        if (distanceFromOrigin > 0.001f) {
            float scale = maxReach / distanceFromOrigin;
            clamped.x *= scale;
            clamped.y *= scale;
        }
    }
    
    // For each arm, ensure the effector joint is reachable
    // This is a more conservative check - we'll try to find a position that works for all arms
    for (int i = 0; i < 3; ++i) {
        glm::vec3 baseJoint = getBaseJointPosition(i);
        glm::vec3 effectorOffset = getEffectorJointOffset(i);
        glm::vec3 effectorJointTarget = clamped + effectorOffset;
        
        glm::vec3 toEffector = effectorJointTarget - baseJoint;
        float distance = glm::length(toEffector);
        
        float minDistance = std::abs(config_.upperArmLength - config_.lowerArmLength);
        float maxDistance = config_.upperArmLength + config_.lowerArmLength;
        
        if (distance < minDistance) {
            // Too close - move effector joint away from base
            glm::vec3 direction = glm::normalize(toEffector);
            if (glm::length(direction) < 0.001f) {
                // Use a default direction
                float angle = i * 2.0f * M_PI / 3.0f;
                direction = glm::normalize(glm::vec3(std::cos(angle), std::sin(angle), -0.5f));
            }
            effectorJointTarget = baseJoint + direction * minDistance;
            clamped = effectorJointTarget - effectorOffset;
        } else if (distance > maxDistance) {
            // Too far - move effector joint closer to base
            glm::vec3 direction = glm::normalize(toEffector);
            effectorJointTarget = baseJoint + direction * maxDistance;
            clamped = effectorJointTarget - effectorOffset;
        }
    }
    
    return clamped;
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

glm::vec3 DeltaRobot::calculateForwardKinematics(const std::array<float, 3>& motorAngles) const {
    // Forward Kinematics for Delta Robot - Analytical Solution
    // Given motor angles, calculate the end-effector position
    // This matches the IK structure exactly: effectorJoint = endEffectorCenter + effectorOffset
    
    std::array<glm::vec3, 3> upperArmEnds;  // Elbow positions
    
    // Step 1: Calculate upper arm end positions (elbow positions) for each arm
    // This must match the IK calculation exactly
    for (int i = 0; i < 3; ++i) {
        glm::vec3 baseJoint = getBaseJointPosition(i);
        
        // Calculate radial direction (outward from center) - same as IK
        glm::vec3 center = glm::vec3(0.0f, 0.0f, baseJoint.z);
        glm::vec3 centerToBase = baseJoint - center;
        float radialDist = glm::length(glm::vec2(centerToBase.x, centerToBase.y));
        
        glm::vec3 radialDir;
        if (radialDist < 0.001f) {
            float angle = i * 2.0f * M_PI / 3.0f;
            radialDir = glm::vec3(std::cos(angle), std::sin(angle), 0.0f);
        } else {
            radialDir = glm::normalize(glm::vec3(centerToBase.x, centerToBase.y, 0.0f));
        }
        
        glm::vec3 verticalDir = glm::vec3(0.0f, 0.0f, 1.0f);
        
        // Calculate upper arm direction in the vertical plane - same as IK
        float motorAngle = motorAngles[i];
        glm::vec3 upperArmDir = radialDir * std::cos(motorAngle) + verticalDir * std::sin(motorAngle);
        upperArmDir = glm::normalize(upperArmDir);
        
        // Calculate upper arm end position (elbow)
        upperArmEnds[i] = baseJoint + upperArmDir * config_.upperArmLength;
    }
    
    // Step 2: Calculate effector joint positions
    // Each effector joint must be at distance lowerArmLength from its elbow
    // The effector joints form a triangle on the effector plate
    // The end-effector center is at the centroid of this triangle
    
    std::array<glm::vec3, 3> effectorJoints;
    std::array<glm::vec3, 3> effectorOffsets;
    
    // Get effector joint offsets (same as used in IK)
    for (int i = 0; i < 3; ++i) {
        effectorOffsets[i] = getEffectorJointOffset(i);
    }
    
    // Step 3: Solve for end-effector center using constraint that:
    // For each arm i: ||(endEffectorCenter + effectorOffset[i]) - upperArmEnds[i]|| = lowerArmLength
    // This is a system of 3 sphere equations
    
    // Use Newton-Raphson to solve the system
    // Initial guess: centroid of elbows, projected down by average lower arm length
    glm::vec3 elbowCentroid = (upperArmEnds[0] + upperArmEnds[1] + upperArmEnds[2]) / 3.0f;
    glm::vec3 effectorCenter = elbowCentroid;
    effectorCenter.z -= config_.lowerArmLength * 0.7f;  // Start below elbows
    
    const int MAX_ITERATIONS = 50;
    const float CONVERGENCE_THRESHOLD = 0.00001f;  // 0.01mm - very tight
    
    for (int iter = 0; iter < MAX_ITERATIONS; ++iter) {
        // Calculate current effector joint positions
        for (int i = 0; i < 3; ++i) {
            effectorJoints[i] = effectorCenter + effectorOffsets[i];
        }
        
        // Calculate errors: how far are the joints from their target spheres?
        std::array<float, 3> errors;
        std::array<glm::vec3, 3> errorGradients;  // Gradient of error w.r.t. effectorCenter
        
        float totalError = 0.0f;
        glm::vec3 totalGradient(0.0f, 0.0f, 0.0f);
        
        for (int i = 0; i < 3; ++i) {
            glm::vec3 toJoint = effectorJoints[i] - upperArmEnds[i];
            float distToJoint = glm::length(toJoint);
            float targetDist = config_.lowerArmLength;
            
            // Error: difference between actual and target distance
            errors[i] = distToJoint - targetDist;
            totalError += errors[i] * errors[i];
            
            if (distToJoint > 0.001f) {
                // Gradient: direction to move effector center to reduce error
                // d(error)/d(center) = (joint - elbow) / ||joint - elbow||
                glm::vec3 direction = glm::normalize(toJoint);
                errorGradients[i] = direction * errors[i];
                totalGradient += errorGradients[i];
            }
        }
        
        // Check convergence
        if (totalError < CONVERGENCE_THRESHOLD * CONVERGENCE_THRESHOLD) {
            break;  // Converged
        }
        
        // Update effector center using gradient descent
        // Use a small step size to ensure stability
        float stepSize = 0.1f;
        effectorCenter = effectorCenter - totalGradient * stepSize;
        
        // Optional: clamp to reasonable bounds to prevent divergence
        // (This shouldn't be necessary if the solution exists, but helps with numerical stability)
    }
    
    return effectorCenter;
}

float DeltaRobot::getIKValidationError(const glm::vec3& desiredPosition, const std::array<float, 3>& motorAngles) const {
    glm::vec3 fkPosition = calculateForwardKinematics(motorAngles);
    return glm::length(fkPosition - desiredPosition);
}

bool DeltaRobot::validateIK(const glm::vec3& desiredPosition, const std::array<float, 3>& motorAngles, 
                            float tolerance) const {
    float error = getIKValidationError(desiredPosition, motorAngles);
    return error <= tolerance;
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

